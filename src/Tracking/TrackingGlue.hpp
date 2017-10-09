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

#ifndef XCSOAR_TRACK_THREAD_HPP
#define XCSOAR_TRACK_THREAD_HPP

#include "Tracking/Features.hpp"

#ifdef HAVE_TRACKING

#include "Tracking/TrackingSettings.hpp"
#include "Tracking/SkyLines/Handler.hpp"
#include "Tracking/SkyLines/Glue.hpp"
#include "Tracking/SkyLines/Data.hpp"
#include "Thread/StandbyThread.hpp"
#include "Tracking/LiveTrack24.hpp"
#include "Time/PeriodClock.hpp"
#include "Geo/GeoPoint.hpp"
#include "Time/BrokenDateTime.hpp"

struct MoreData;
struct DerivedInfo;

class TrackingGlue final
#if defined(HAVE_LIVETRACK24) || defined(HAVE_SKYLINES_TRACKING_HANDLER)
  :
#endif
#ifdef HAVE_LIVETRACK24
  protected StandbyThread
#endif
#if defined(HAVE_LIVETRACK24) && defined(HAVE_SKYLINES_TRACKING_HANDLER)
  ,
#endif
#ifdef HAVE_SKYLINES_TRACKING_HANDLER
  private SkyLinesTracking::Handler
#endif
{
  struct LiveTrack24State
  {
    LiveTrack24::SessionID session_id;
    unsigned packet_id;

    void ResetSession() {
      session_id = 0;
    }

    bool HasSession() {
      return session_id != 0;
    }
  };

  PeriodClock clock;

  TrackingSettings settings;

#ifdef HAVE_SKYLINES_TRACKING
  SkyLinesTracking::Glue skylines;

#ifdef HAVE_SKYLINES_TRACKING_HANDLER
  SkyLinesTracking::Data skylines_data;
#endif
#endif

#ifdef HAVE_LIVETRACK24
  LiveTrack24State state;

  /**
   * The Unix UTC time stamp that was last submitted to the tracking
   * server.  This attribute is used to detect time warps.
   */
  int64_t last_timestamp;

  BrokenDateTime date_time;
  GeoPoint location;
  unsigned altitude;
  unsigned ground_speed;
  Angle track;
  bool flying, last_flying;
#endif

public:
  TrackingGlue();

#ifdef HAVE_LIVETRACK24
  void StopAsync();
  void WaitStopped();
#else
  void StopAsync() {}
  void WaitStopped() {}
#endif

  void SetSettings(const TrackingSettings &_settings);
  void OnTimer(const MoreData &basic, const DerivedInfo &calculated);

#ifdef HAVE_LIVETRACK24
protected:
  void Tick() override;
#endif

#ifdef HAVE_SKYLINES_TRACKING_HANDLER
private:
  /* virtual methods from SkyLinesTracking::Handler */
  virtual void OnTraffic(uint32_t pilot_id, unsigned time_of_day_ms,
                         const GeoPoint &location, int altitude) override;
    virtual void OnUserName(uint32_t user_id, const TCHAR *name) override;
  void OnWave(unsigned time_of_day_ms,
              const GeoPoint &a, const GeoPoint &b) override;

public:
  const SkyLinesTracking::Data &GetSkyLinesData() const {
    return skylines_data;
  }
#endif
};

#endif /* HAVE_TRACKING */
#endif
