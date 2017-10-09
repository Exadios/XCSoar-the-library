/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Terrain/RasterTileCache.hpp"
#include "Terrain/RasterLocation.hpp"
#include "jasper/jas_image.h"
#include "Math/Angle.hpp"
#include "IO/ZipLineReader.hpp"
#include "Operation/Operation.hpp"
#include "Math/FastMath.h"

#include <string.h>
#include <algorithm>

short*
RasterTileCache::GetImageBuffer(unsigned index)
{
  if (TileRequest(index))
    return tiles.GetLinear(index).GetImageBuffer();

  return NULL;
}

void
RasterTileCache::SetTile(unsigned index,
                         int xstart, int ystart, int xend, int yend)
{
  if (!segments.empty() && !segments.last().IsTileSegment())
    /* link current marker segment with this tile */
    segments.last().tile = index;

  tiles.GetLinear(index).Set(xstart, ystart, xend, yend);
}

struct RTDistanceSort {
  const RasterTileCache &rtc;

  RTDistanceSort(RasterTileCache &_rtc):rtc(_rtc) {}

  bool operator()(unsigned short ai, unsigned short bi) const {
    const RasterTile &a = rtc.tiles.GetLinear(ai);
    const RasterTile &b = rtc.tiles.GetLinear(bi);

    return a.GetDistance() < b.GetDistance();
  }
};

bool
RasterTileCache::PollTiles(int x, int y, unsigned radius)
{
  if (scan_overview)
    return false;

  /* tiles are usually 256 pixels wide; with a radius smaller than
     that, the (optimized) tile distance calculations may fail;
     additionally, this ensures that tiles which are slightly out of
     the screen will be loaded in advance */
  radius += 256;

  /**
   * Maximum number of tiles loaded at a time, to reduce system load
   * peaks.
   */
  constexpr unsigned MAX_ACTIVATE = MAX_ACTIVE_TILES > 32
    ? 16
    : MAX_ACTIVE_TILES / 2;

  /* query all tiles; all tiles which are either in range or already
     loaded are added to RequestTiles */

  request_tiles.clear();
  for (int i = tiles.GetSize() - 1; i >= 0 && !request_tiles.full(); --i)
    if (tiles.GetLinear(i).VisibilityChanged(x, y, radius))
      request_tiles.append(i);

  /* reduce if there are too many */

  if (request_tiles.size() > MAX_ACTIVE_TILES) {
    /* sort by distance */
    const RTDistanceSort sort(*this);
    std::sort(request_tiles.begin(), request_tiles.end(), sort);

    /* dispose all tiles which are out of range */
    for (unsigned i = MAX_ACTIVE_TILES; i < request_tiles.size(); ++i) {
      RasterTile &tile = tiles.GetLinear(request_tiles[i]);
      tile.Disable();
    }

    request_tiles.shrink(MAX_ACTIVE_TILES);
  }

  /* fill ActiveTiles and request new tiles */

  dirty = false;

  unsigned num_activate = 0;
  for (unsigned i = 0; i < request_tiles.size(); ++i) {
    RasterTile &tile = tiles.GetLinear(request_tiles[i]);
    if (tile.IsEnabled())
      continue;

    if (++num_activate <= MAX_ACTIVATE)
      /* request the tile in the current iteration */
      tile.SetRequest();
    else
      /* this tile will be loaded in the next iteration */
      dirty = true;
  }

  return num_activate > 0;
}

bool
RasterTileCache::TileRequest(unsigned index)
{
  RasterTile &tile = tiles.GetLinear(index);

  if (!tile.IsRequested())
    return false;

  tile.Enable();
  return true; // want to load this one!
}

short
RasterTileCache::GetHeight(unsigned px, unsigned py) const
{
  if (px >= width || py >= height)
    // outside overall bounds
    return RasterBuffer::TERRAIN_INVALID;

  const RasterTile &tile = tiles.Get(px / tile_width, py / tile_height);
  if (tile.IsEnabled())
    return tile.GetHeight(px, py);

  // still not found, so go to overview
  return overview.GetInterpolated(px << (SUBPIXEL_BITS - OVERVIEW_BITS),
                                   py << (SUBPIXEL_BITS - OVERVIEW_BITS));
}

short
RasterTileCache::GetInterpolatedHeight(unsigned int lx, unsigned int ly) const
{
  if ((lx >= overview_width_fine) || (ly >= overview_height_fine))
    // outside overall bounds
    return RasterBuffer::TERRAIN_INVALID;

  unsigned px = lx, py = ly;
  const unsigned int ix = CombinedDivAndMod(px);
  const unsigned int iy = CombinedDivAndMod(py);

  const RasterTile &tile = tiles.Get(px / tile_width, py / tile_height);
  if (tile.IsEnabled())
    return tile.GetInterpolatedHeight(px, py, ix, iy);

  // still not found, so go to overview
  return overview.GetInterpolated(lx >> OVERVIEW_BITS,
                                   ly >> OVERVIEW_BITS);
}

void
RasterTileCache::SetSize(unsigned _width, unsigned _height,
                         unsigned _tile_width, unsigned _tile_height,
                         unsigned tile_columns, unsigned tile_rows)
{
  width = _width;
  height = _height;
  tile_width = _tile_width;
  tile_height = _tile_height;

  overview.Resize(width >> OVERVIEW_BITS, height >> OVERVIEW_BITS);
  overview_width_fine = width << SUBPIXEL_BITS;
  overview_height_fine = height << SUBPIXEL_BITS;

  tiles.GrowDiscard(tile_columns, tile_rows);
}

void
RasterTileCache::SetLatLonBounds(double _lon_min, double _lon_max,
                                 double _lat_min, double _lat_max)
{
  const Angle lon_min(Angle::Degrees(_lon_min));
  const Angle lon_max(Angle::Degrees(_lon_max));
  const Angle lat_min(Angle::Degrees(_lat_min));
  const Angle lat_max(Angle::Degrees(_lat_max));

  bounds = GeoBounds(GeoPoint(std::min(lon_min, lon_max),
                              std::max(lat_min, lat_max)),
                     GeoPoint(std::max(lon_min, lon_max),
                              std::min(lat_min, lat_max)));
  bounds_initialised = true;
}

void
RasterTileCache::Reset()
{
  width = 0;
  height = 0;
  initialised = false;
  bounds_initialised = false;
  segments.clear();
  scan_overview = true;

  overview.Reset();

  for (auto it = tiles.begin(), end = tiles.end(); it != end; ++it)
    it->Disable();
}

gcc_pure
const RasterTileCache::MarkerSegmentInfo *
RasterTileCache::FindMarkerSegment(uint32_t file_offset) const
{
  for (const MarkerSegmentInfo *p = segments.begin(); p < segments.end(); ++p)
    if (p->file_offset >= file_offset)
      return p;

  return NULL;
}

long
RasterTileCache::SkipMarkerSegment(long file_offset) const
{
  if (scan_overview)
    /* use all segments when loading the overview */
    return 0;

  if (remaining_segments > 0) {
    /* enable the follow-up segment */
    --remaining_segments;
    return 0;
  }

  const MarkerSegmentInfo *segment = FindMarkerSegment(file_offset);
  if (segment == NULL)
    /* past the end of the recorded segment list; shouldn't happen */
    return 0;

  long skip_to = segment->file_offset;
  while (segment->IsTileSegment() &&
         !tiles.GetLinear(segment->tile).IsRequested()) {
    ++segment;
    if (segment >= segments.end())
      /* last segment is hidden; shouldn't happen either, because we
         expect EOC there */
      break;

    skip_to = segment->file_offset;
  }

  remaining_segments = segment->count;
  return skip_to - file_offset;
}

/**
 * Does this segment belong to the preceding tile?  If yes, then it
 * inherits the tile number.
 */
static bool
is_tile_segment(unsigned id)
{
  return id == 0xff93 /* SOD */ ||
    id == 0xff52 /* COD */ ||
    id == 0xff53 /* COC */ ||
    id == 0xff5c /* QCD */ ||
    id == 0xff5d /* QCC */ ||
    id == 0xff5e /* RGN */ ||
    id == 0xff5f /* POC */ ||
    id == 0xff61 /* PPT */ ||
    id == 0xff58 /* PLT */ ||
    id == 0xff64 /* COM */;
}

void
RasterTileCache::MarkerSegment(long file_offset, unsigned id)
{
  if (!scan_overview || segments.full())
    return;

  if (operation != NULL)
    operation->SetProgressPosition(file_offset / 65536);

  if (is_tile_segment(id) && !segments.empty() &&
      segments.last().IsTileSegment()) {
    /* this segment belongs to the same tile as the preceding SOT
       segment */
    ++segments.last().count;
    return;
  }

  if (segments.size() >= 2 && !segments.last().IsTileSegment() &&
      !segments[segments.size() - 2].IsTileSegment()) {
    /* the last two segments are both "generic" segments and can be merged*/
    assert(segments.last().count == 0);

    ++segments[segments.size() - 2].count;

    /* reuse the second segment */
    segments.last().file_offset = file_offset;
  } else
    segments.append(MarkerSegmentInfo(file_offset,
                                      MarkerSegmentInfo::NO_TILE));
}

extern RasterTileCache *raster_tile_current;

void
RasterTileCache::LoadJPG2000(const char *jp2_filename)
{
  jas_stream_t *in;

  raster_tile_current = this;

  in = jas_stream_fopen(jp2_filename, "rb");
  if (!in) {
    Reset();
    return;
  }

  if (operation != NULL)
    operation->SetProgressRange(jas_stream_length(in) / 65536);

  jp2_decode(in, scan_overview ? "xcsoar=2" : "xcsoar=1");
  jas_stream_close(in);
}

bool
RasterTileCache::LoadWorldFile(const TCHAR *path)
{
  ZipLineReaderA reader(path);
  if (reader.error())
    return false;

  char *endptr;
  const char *line = reader.ReadLine(); // x scale
  double x_scale = strtod(line, &endptr);
  if (endptr == line)
    return false;

  line = reader.ReadLine(); // y rotation
  if (line == NULL)
    return false;

  double y_rotation = strtod(line, &endptr);
  if (endptr == line || y_rotation < -0.01 || y_rotation > 0.01)
    /* we don't support rotation */
    return false;

  line = reader.ReadLine(); // x rotation
  if (line == NULL)
    return false;

  double x_rotation = strtod(line, &endptr);
  if (endptr == line || x_rotation < -0.01 || x_rotation > 0.01)
    /* we don't support rotation */
    return false;

  line = reader.ReadLine(); // y scale
  if (line == NULL)
    return false;

  double y_scale = strtod(line, &endptr);
  if (endptr == line)
    return false;

  line = reader.ReadLine(); // x origin
  if (line == NULL)
    return false;

  double x_origin = strtod(line, &endptr);
  if (endptr == line)
    return false;

  line = reader.ReadLine(); // y origin
  if (line == NULL)
    return false;

  double y_origin = strtod(line, &endptr);
  if (endptr == line)
    return false;

  SetLatLonBounds(x_origin, x_origin + GetWidth() * x_scale,
                  y_origin, y_origin + GetHeight() * y_scale);
  return true;
}

bool
RasterTileCache::LoadOverview(const char *path, const TCHAR *world_file,
                              OperationEnvironment &_operation)
{
  assert(operation == NULL);
  operation = &_operation;

  Reset();

  LoadJPG2000(path);
  scan_overview = false;

  if (initialised && world_file != NULL)
    LoadWorldFile(world_file);

  if (initialised && !bounds_initialised)
    initialised = false;

  if (!initialised)
    Reset();

  operation = NULL;
  return initialised;
}

void
RasterTileCache::UpdateTiles(const char *path, int x, int y, unsigned radius)
{
  if (!PollTiles(x, y, radius))
    return;

  remaining_segments = 0;

  LoadJPG2000(path);

  /* permanently disable the requested tiles which are still not
     loaded, to prevent trying to reload them over and over in a busy
     loop */
  for (auto it = request_tiles.begin(), end = request_tiles.end();
      it != end; ++it) {
    RasterTile &tile = tiles.GetLinear(*it);
    if (tile.IsRequested() && !tile.IsEnabled())
      tile.Clear();
  }

  ++serial;
}

bool
RasterTileCache::SaveCache(FILE *file) const
{
  if (!initialised)
    return false;

  assert(bounds_initialised);

  /* save metadata */
  CacheHeader header;

  /* zero-fill all implicit padding bytes (to make valgrind happy) */
  memset(&header, 0, sizeof(header));

  header.version = CacheHeader::VERSION;
  header.width = width;
  header.height = height;
  header.tile_width = tile_width;
  header.tile_height = tile_height;
  header.tile_columns = tiles.GetWidth();
  header.tile_rows = tiles.GetHeight();
  header.num_marker_segments = segments.size();
  header.bounds = bounds;

  if (fwrite(&header, sizeof(header), 1, file) != 1 ||
      /* .. and segments */
      fwrite(segments.begin(), sizeof(*segments.begin()), segments.size(), file) != segments.size())
    return false;

  /* save tiles */
  unsigned i;
  for (i = 0; i < tiles.GetSize(); ++i)
    if (tiles.GetLinear(i).IsDefined() &&
        (fwrite(&i, sizeof(i), 1, file) != 1 ||
         !tiles.GetLinear(i).SaveCache(file)))
      return false;

  i = -1;
  if (fwrite(&i, sizeof(i), 1, file) != 1)
    return false;

  /* save overview */
  size_t overview_size = overview.GetWidth() * overview.GetHeight();
  if (fwrite(overview.GetData(), sizeof(*overview.GetData()),
             overview_size, file) != overview_size)
    return false;

  /* done */
  return true;
}

bool
RasterTileCache::LoadCache(FILE *file)
{
  Reset();

  /* load metadata */
  CacheHeader header;
  if (fread(&header, sizeof(header), 1, file) != 1 ||
      header.version != CacheHeader::VERSION ||
      header.width < 1024 || header.width > 1024 * 1024 ||
      header.height < 1024 || header.height > 1024 * 1024 ||
      header.tile_width < 16 || header.tile_width > 16 * 1024 ||
      header.tile_height < 16 || header.tile_height > 16 * 1024 ||
      header.tile_columns < 1 || header.tile_columns > 1024 ||
      header.tile_rows < 1 || header.tile_rows > 1024 ||
      header.num_marker_segments < 4 ||
      header.num_marker_segments > segments.capacity() ||
      header.bounds.IsEmpty())
    return false;

  SetSize(header.width, header.height,
          header.tile_width, header.tile_height,
          header.tile_columns, header.tile_rows);
  bounds = header.bounds;
  bounds_initialised = true;

  /* load segments */
  for (unsigned i = 0; i < header.num_marker_segments; ++i) {
    MarkerSegmentInfo &segment = segments.append();
    if (fread(&segment, sizeof(segment), 1, file) != 1)
      return false;
  }

  /* load tiles */
  unsigned i;
  while (true) {
    if (fread(&i, sizeof(i), 1, file) != 1)
      return false;

    if (i == (unsigned)-1)
      break;

    if (i >= tiles.GetSize())
      return false;

    if (!tiles.GetLinear(i).LoadCache(file))
      return false;
  }

  /* load overview */
  size_t overview_size = overview.GetWidth() * overview.GetHeight();
  if (fread(overview.GetData(), sizeof(*overview.GetData()),
            overview_size, file) != overview_size)
    return false;

  initialised = true;
  scan_overview = false;
  return true;
}
