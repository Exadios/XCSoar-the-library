set(XCSOAR_DATA ${XCSOAR}/Data)
set(GRAPHICS_SRC_DIR ${XCSOAR_DATA}/graphics)
set(BITMAP_SRC_DIR   ${XCSOAR_DATA}/bitmaps)
set(ICONS_SRC_DIR    ${XCSOAR_DATA}/icons)
set(SOUNDS_SRC_DIR   ${XCSOAR_DATA}/sound)

set(OUT_DATA  ${XCSoar_BINARY_DIR}/data)
set(GRAPHICS_DST_DIR ${OUT_DATA}/graphics)
set(BITMAP_DST_DIR   ${OUT_DATA}/bitmaps)
set(ICONS_DST_DIR    ${OUT_DATA}/icons)
set(SOUNDS_DST_DIR   ${OUT_DATA}/sound)

set(fr "")

# Market icons
set(MARKET_ICONS logo logo_red)
foreach(icon IN LISTS MARKET_ICONS)
  set(fqs ${GRAPHICS_SRC_DIR}/${icon})
  set(fqd ${ICONS_DST_DIR}/${icon})
  add_custom_command(OUTPUT ${fqd}.png
                     COMMAND rsvg-convert --width 512 ${fqs}.svg -o ${fqd}.png
                    )
  set(fr ${fr} ${fqd}.png)
endforeach(icon)

# Bitmaps
set(BITMAPS aboveterrain
            airspace
            airspace0
            airspace1
            airspace2
            airspace3
            airspace4
            airspace5
            airspace6
            airspace7
            climb_12
            climb_12inv
            vario_scale_b
            vario_scale_c
   )
foreach(bitmap IN LISTS BITMAPS)
  add_custom_command(OUTPUT ${BITMAP_DST_DIR}/${bitmap}.png
                     COMMAND convert +dither -type GrayScale -define png:color-type=0 ${BITMAP_SRC_DIR}/${bitmap}.bmp ${BITMAP_DST_DIR}/${bitmap}.png
                    )
  set(fr ${fr} ${BITMAP_DST_DIR}/${bitmap}.png)
endforeach(bitmap)

# Icons
set(ICONS airspace_intercept
          alt2_landable_airport
          alt2_landable_field
          alt2_marginal_airport
          alt2_marginal_field
          alt_landable_airport
          alt_landable_field
          alt_marginal_airport
          alt_marginal_field
          alt_reachable_airport
          alt_reachable_field
          calculator
          clock
          device
          flarm_alarm
          flarm_traffic
          flarm_warning
          folder
          globe
          gps_acquiring
          gps_disconnected
          gps_lowsignal
          instrument_vario1
          instrument_vario2
          map_bridge
          map_flag
          map_mountain_top
          map_obstacle
          map_pass
          map_power_plant
          map_small
          map_target
          map_taskturnpoint
          map_teammate
          map_terrainw
          map_thermal_hotspot
          map_thermal_source
          map_tower
          map_town
          map_tunnel
          map_turnpoint
          map_weather_station
          mode_abort
          mode_climb
          mode_cruise
          mode_finalglide
          rules
          scalearrow_left
          scalearrow_right
          settings
          task
          vario1
          winpilot_landable
          winpilot_marginal
          winpilot_reachable
          wrench
   )
foreach(svg_noalias_icon IN LISTS ICONS)
  add_custom_command(OUTPUT ${ICONS_DST_DIR}/${svg_noalias_icon}.svg
                     COMMAND xsltproc --stringparam DisableAA_Select "MASK_NOAA_" --output ${ICONS_DST_DIR}/${svg_noalias_icon}.svg ${XCSOAR_BUILD}/svg_preprocess.xsl ${ICONS_SRC_DIR}/${svg_noalias_icon}.svg
                     DEPENDS ${XCSOAR_BUILD}/svg_preprocess.xsl
                    )
  set(fr ${fr} ${ICONS_DST_DIR}/${svg_noalias_icon}.svg)
endforeach(svg_noalias_icon)

# Make the png, alpha, rgb and tile images
foreach(icon IN LISTS ICONS)
  set(fqs ${ICONS_SRC_DIR}/${icon})
  set(fqd ${ICONS_DST_DIR}/${icon})
  add_custom_command(OUTPUT ${fqd}.png
                            ${fqd}_160.png
                            ${fqd}_alpha.png
                            ${fqd}_rgb.png
                            ${fqd}_tile.png
                            ${fqd}.bmp
                            ${fqd}_160.bmp
  # Render from SVG to PNG
  # Default 100PPI (eg 320x240 4" display)
                     COMMAND rsvg-convert --x-zoom=1.0 --y-zoom=1.0 ${fqs}.svg -o ${fqd}.png
  # 160PPI (eg 640x480 5" display)
                     COMMAND rsvg-convert --x-zoom=1.6316 --y-zoom=1.6316 ${fqs}.svg -o ${fqd}_160.png
                     COMMAND convert ${fqd}.png -alpha Extract +matte +dither -colors 8 ${fqd}_alpha.png
                     COMMAND convert ${fqd}.png -background white -flatten +matte +dither -colors 64 ${fqd}_rgb.png
                     COMMAND montage -tile 2x1 -geometry +0+0 ${fqd}_alpha.png ${fqd}_rgb.png -depth 8 ${fqd}_tile.png
  # convert to uncompressed 8-bit BMP
                     COMMAND convert ${fqd}_tile.png +dither -compress none -type optimize -colors 256 bmp3:${fqd}.bmp
                     COMMAND convert ${fqd}_tile.png +dither -compress none -type optimize -colors 256 bmp3:${fqd}_160.bmp
                    DEPENDS ${fqd}.svg
                    )
  set(fr ${fr} ${fqd}.png ${fqd}_160.png ${fqd}_alpha.png ${fqd}_rgb.png ${fqd}_tile.png ${fqd}.bmp ${fqd}_160.bmp)
endforeach(icon)

####### splash logo

set(SPLASH logo logo_red)
foreach(splash IN LISTS SPLASH)
  set(fqs ${GRAPHICS_SRC_DIR}/${splash})
  set(fqd ${GRAPHICS_DST_DIR}/${splash})
  add_custom_command(OUTPUT ${fqd}_320.png
                            ${fqd}_160.png
                            ${fqd}_80.png
                            ${fqd}_1024.png
                            ${fqd}_160.bmp
                            ${fqd}_80.bmp
                     COMMAND rsvg-convert --width=320 ${fqs}.svg -o ${fqd}_320.png
                     COMMAND rsvg-convert --width=160 ${fqs}.svg -o ${fqd}_160.png
                     COMMAND rsvg-convert --width=80  ${fqs}.svg -o ${fqd}_80.png
                     COMMAND rsvg-convert --width=1024 ${fqs}.svg -o ${fqd}_1024.png
                     COMMAND convert ${fqd}_160.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}_160.bmp
                     COMMAND convert ${fqd}_80.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}_80.bmp
                    )
  set(fr ${fr} ${fqd}_320.png ${fqd}_160.png ${fqd}_80.png ${fqd}_1024.png
               ${fqd}_160.bmp ${fqd}_80.png)
endforeach(splash)

####### version

set(VERSION title title_red)
foreach(version IN LISTS VERSION)
  set(fqs ${GRAPHICS_SRC_DIR}/${version})
  set(fqd ${GRAPHICS_DST_DIR}/${version})
  add_custom_command(OUTPUT ${fqd}_110.png
                            ${fqd}_320.png
                            ${fqd}_110.bmp
                            ${fqd}_320.bmp
                     COMMAND rsvg-convert --width=110 ${fqs}.svg -o ${fqd}_110.png
                     COMMAND rsvg-convert --width=320 ${fqs}.svg -o ${fqd}_320.png
                     COMMAND convert ${fqd}_110.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}_110.bmp
                     COMMAND convert ${fqd}_320.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}_320.bmp
                    )
  set(fr ${fr} ${fqd}_110.png ${fqd}_320.png ${fqd}_110.bmp ${fqd}_320.bmp)
endforeach(version)

####### dialog title and progress bar border

set(FICHA dialog_title dialog_title_red progress_border progress_border_red)
foreach(ficha IN LISTS FICHA)
  set(fqs ${GRAPHICS_SRC_DIR}/${ficha})
  set(fqd ${GRAPHICS_DST_DIR}/${ficha})
  add_custom_command(OUTPUT ${fqd}.png
                            ${fqd}.bmp
                     COMMAND rsvg-convert ${fqs}.svg -o ${fqd}.png
                     COMMAND convert ${fqd}.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}.bmp
                    )
  set(fr ${fr} ${fqd}.png ${fqd}.bmp)
endforeach(ficha)

####### launcher graphics

set(LAUNCH launcher launcher_red)
foreach(launch IN LISTS LAUNCH)
  set(fqs ${GRAPHICS_SRC_DIR}/${launch})
  set(fqd ${GRAPHICS_DST_DIR}/${launch})
  add_custom_command(OUTPUT ${fqd}_224.png
                            ${fqd}_224.bmp
                            ${fqd}_224_1.png
                            ${fqd}_224_2.png
                            ${fqd}_224_1.bmp
                            ${fqd}_224_2.bmp
                     COMMAND rsvg-convert --width=224 ${fqs}.svg -o ${fqd}_224.png
                     COMMAND convert ${fqd}_224.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256  bmp3:${fqd}_224.bmp
                     COMMAND convert ${fqd}_224.png -background white -layers flatten +matte +dither -compress none -type optimize -colors 256 -crop '50%x100%' -scene 1 bmp3:${fqd}_224_%d.bmp
                     COMMAND convert ${fqd}_224_2.bmp ${fqd}_224_2.png
                     COMMAND convert ${fqd}_224_1.bmp ${fqd}_224_1.png
                     )
  set(fr ${fr} ${fqd}_224.png ${fqd}_224.bmp ${fqd}_224_1.png ${fqd}_224_2.png ${fqd}_224_1.bmp ${fqd}_224_2.bmp)
endforeach(launch)

####### sounds

set(SOUNDS beep_bweep
           beep_clear
           beep_drip
           fail
           insert
           remove
   )
foreach(sound IN LISTS SOUNDS)
  set(fqs ${SOUNDS_SRC_DIR}/${sound}.wav)
  set(fqd ${SOUNDS_DST_DIR}/${sound}.raw)
  add_custom_command(OUTPUT ${fqd}
                     COMMAND ffmpeg -y -v 0  -i ${fqs} -f s16le -ar 44100 -ac 1 -acodec pcm_s16le ${fqd}
                    )
  set(fr ${fr} ${fqd})
endforeach(sound)

####### text files

set(TEXT_SRC_DIR ${XCSOAR})
set(TEXT_DST_DIR ${OUT_DATA})
set(TEXT_FILES AUTHORS COPYING)
foreach(text IN LISTS TEXT_FILES)
  add_custom_command(OUTPUT ${TEXT_DST_DIR}/${text}.gz
                     COMMAND gzip --best < ${TEXT_SRC_DIR}/${text} > ${TEXT_DST_DIR}/${text}.gz.tmp
                     COMMAND mv ${TEXT_DST_DIR}/${text}.gz.tmp ${TEXT_DST_DIR}/${text}.gz
                    )
  set(fr ${fr} ${TEXT_DST_DIR}/${text}.gz)
endforeach(text)

# RESOURCE_FILES set from the original make.
# /TODO /todo Write the rules to get RESOURCE_FILES from the data.
set(RESOURCE_FILES 
                   AUTHORS.gz
                   COPYING.gz
                   bitmaps/airspace0.png
                   bitmaps/vario_scale_b.png
                   bitmaps/airspace4.png
                   bitmaps/vario_scale_c.png
                   bitmaps/airspace1.png
                   bitmaps/climb_12.png
                   bitmaps/airspace6.png
                   bitmaps/airspace3.png
                   bitmaps/aboveterrain.png
                   bitmaps/airspace5.png
                   bitmaps/climb_12inv.png
                   bitmaps/airspace2.png
                   bitmaps/airspace.png
                   bitmaps/airspace7.png
                   icons2/device.png
                   icons2/alt_landable_field.png
                   icons2/winpilot_landable.png
                   icons2/map_teammate.png
                   icons2/map_flag.png
                   icons2/mode_cruise.png
                   icons2/folder.png
                   icons2/map_thermal_hotspot.png
                   icons2/alt_reachable_airport.png
                   icons2/vario1.png
                   icons2/map_turnpoint.png
                   icons2/calculator.png
                   icons2/map_target.png
                   icons2/alt2_landable_airport.png
                   icons2/alt_reachable_field.png
                   icons2/winpilot_reachable.png
                   icons2/instrument_vario1.png
                   icons2/map_mountain_top.png
                   icons2/airspace_intercept.png
                   icons2/gps_acquiring.png
                   icons2/scalearrow_right.png
                   icons2/map_thermal_source.png
                   icons2/alt2_landable_field.png
                   icons2/map_terrainw.png
                   icons2/map_bridge.png
                   icons2/mode_climb.png
                   icons2/map_town.png
                   icons2/alt_marginal_airport.png
                   icons2/map_weather_station.png
                   icons2/settings.png
                   icons2/map_obstacle.png
                   icons2/map_power_plant.png
                   icons2/map_tower.png
                   icons2/gps_disconnected.png
                   icons2/instrument_vario2.png
                   icons2/map_pass.png
                   icons2/winpilot_marginal.png
                   icons2/scalearrow_left.png
                   icons2/flarm_warning.png
                   icons2/mode_abort.png
                   icons2/map_small.png
                   icons2/wrench.png
                   icons2/flarm_traffic.png
                   icons2/globe.png
                   icons2/gps_lowsignal.png
                   icons2/alt2_marginal_airport.png
                   icons2/clock.png
                   icons2/rules.png
                   icons2/flarm_alarm.png
                   icons2/alt_marginal_field.png
                   icons2/alt_landable_airport.png
                   icons2/mode_finalglide.png
                   icons2/task.png
                   icons2/map_taskturnpoint.png
                   icons2/map_tunnel.png
                   icons2/alt2_marginal_field.png
                   icons2/device_160.png
                   icons2/alt_landable_field_160.png
                   icons2/winpilot_landable_160.png
                   icons2/map_teammate_160.png
                   icons2/map_flag_160.png
                   icons2/mode_cruise_160.png
                   icons2/folder_160.png
                   icons2/map_thermal_hotspot_160.png
                   icons2/alt_reachable_airport_160.png
                   icons2/vario1_160.png
                   icons2/map_turnpoint_160.png
                   icons2/calculator_160.png
                   icons2/map_target_160.png
                   icons2/alt2_landable_airport_160.png
                   icons2/alt_reachable_field_160.png
                   icons2/winpilot_reachable_160.png
                   icons2/instrument_vario1_160.png
                   icons2/map_mountain_top_160.png
                   icons2/airspace_intercept_160.png
                   icons2/gps_acquiring_160.png
                   icons2/scalearrow_right_160.png
                   icons2/map_thermal_source_160.png
                   icons2/alt2_landable_field_160.png
                   icons2/map_terrainw_160.png
                   icons2/map_bridge_160.png
                   icons2/mode_climb_160.png
                   icons2/map_town_160.png
                   icons2/alt_marginal_airport_160.png
                   icons2/map_weather_station_160.png
                   icons2/settings_160.png
                   icons2/map_obstacle_160.png
                   icons2/map_power_plant_160.png
                   icons2/map_tower_160.png
                   icons2/gps_disconnected_160.png
                   icons2/instrument_vario2_160.png
                   icons2/map_pass_160.png
                   icons2/winpilot_marginal_160.png
                   icons2/scalearrow_left_160.png
                   icons2/flarm_warning_160.png
                   icons2/mode_abort_160.png
                   icons2/map_small_160.png
                   icons2/wrench_160.png
                   icons2/flarm_traffic_160.png
                   icons2/globe_160.png
                   icons2/gps_lowsignal_160.png
                   icons2/alt2_marginal_airport_160.png
                   icons2/clock_160.png
                   icons2/rules_160.png
                   icons2/flarm_alarm_160.png
                   icons2/alt_marginal_field_160.png
                   icons2/alt_landable_airport_160.png
                   icons2/mode_finalglide_160.png
                   icons2/task_160.png
                   icons2/map_taskturnpoint_160.png
                   icons2/map_tunnel_160.png
                   icons2/alt2_marginal_field_160.png
                   graphics2/logo_160.png
                   graphics2/logo_red_160.png
                   graphics2/logo_80.png
                   graphics2/logo_red_80.png
                   graphics2/dialog_title.png
                   graphics2/dialog_title_red.png
                   graphics2/progress_border.png
                   graphics2/progress_border_red.png
                   graphics2/title_320.png
                   graphics2/title_red_320.png
                   graphics2/title_110.png
                   graphics2/title_red_110.png
                   graphics2/launcher_224_1.png
                   graphics2/launcher_red_224_1.png
                   graphics2/launcher_224_2.png
                   graphics2/launcher_red_224_2.png
                   sound/fail.raw
                   sound/remove.raw
                   sound/beep_drip.raw
                   sound/beep_bweep.raw
                   sound/beep_clear.raw
                   sound/insert.raw
                  )
set(ICONS2         device
                   alt_landable_field
                   winpilot_landable
                   map_teammate
                   map_flag
                   mode_cruise
                   folder
                   map_thermal_hotspot
                   alt_reachable_airport
                   vario1
                   map_turnpoint
                   calculator
                   map_target
                   alt2_landable_airport
                   alt_reachable_field
                   winpilot_reachable
                   instrument_vario1
                   map_mountain_top
                   airspace_intercept
                   gps_acquiring
                   scalearrow_right
                   map_thermal_source
                   alt2_landable_field
                   map_terrainw
                   map_bridge
                   mode_climb
                   map_town
                   alt_marginal_airport
                   map_weather_station
                   settings
                   map_obstacle
                   map_power_plant
                   map_tower
                   gps_disconnected
                   instrument_vario2
                   map_pass
                   winpilot_marginal
                   scalearrow_left
                   flarm_warning
                   mode_abort
                   map_small
                   wrench
                   flarm_traffic
                   globe
                   gps_lowsignal
                   alt2_marginal_airport
                   clock
                   rules
                   flarm_alarm
                   alt_marginal_field
                   alt_landable_airport
                   mode_finalglide
                   task
                   map_taskturnpoint
                   map_tunnel
                   alt2_marginal_field
                   device_160
                   alt_landable_field_160
                   winpilot_landable_160
                   map_teammate_160
                   map_flag_160
                   mode_cruise_160
                   folder_160
                   map_thermal_hotspot_160
                   alt_reachable_airport_160
                   vario1_160
                   map_turnpoint_160
                   calculator_160
                   map_target_160
                   alt2_landable_airport_160
                   alt_reachable_field_160
                   winpilot_reachable_160
                   instrument_vario1_160
                   map_mountain_top_160
                   airspace_intercept_160
                   gps_acquiring_160
                   scalearrow_right_160
                   map_thermal_source_160
                   alt2_landable_field_160
                   map_terrainw_160
                   map_bridge_160
                   mode_climb_160
                   map_town_160
                   alt_marginal_airport_160
                   map_weather_station_160
                   settings_160
                   map_obstacle_160
                   map_power_plant_160
                   map_tower_160
                   gps_disconnected_160
                   instrument_vario2_160
                   map_pass_160
                   winpilot_marginal_160
                   scalearrow_left_160
                   flarm_warning_160
                   mode_abort_160
                   map_small_160
                   wrench_160
                   flarm_traffic_160
                   globe_160
                   gps_lowsignal_160
                   alt2_marginal_airport_160
                   clock_160
                   rules_160
                   flarm_alarm_160
                   alt_marginal_field_160
                   alt_landable_airport_160
                   mode_finalglide_160
                   task_160
                   map_taskturnpoint_160
                   map_tunnel_160
                   alt2_marginal_field_160
   )
set(GRAPHICS2      logo_160
                   logo_red_160
                   logo_80
                   logo_red_80
                   dialog_title
                   dialog_title_red
                   progress_border
                   progress_border_red
                   title_320
                   title_red_320
                   title_110
                   title_red_110
                   launcher_224_1
                   launcher_red_224_1
                   launcher_224_2
                   launcher_red_224_2
   )
foreach(icon IN LISTS ICONS2)
  add_custom_command(OUTPUT ${OUT_DATA}/icons2/${icon}.png
                     COMMAND convert ${OUT_DATA}/icons/${icon}.bmp ${OUT_DATA}/icons2/${icon}.png
                     DEPENDS ${OUT_DATA}/icons/${icon}.bmp
                    )
  set(fr ${fr} ${OUT_DATA}/icons2/${icon}.png)
endforeach(icon)
foreach(graphic IN LISTS GRAPHICS2)
  add_custom_command(OUTPUT ${OUT_DATA}/graphics2/${graphic}.png
                     COMMAND convert ${OUT_DATA}/graphics/${graphic}.bmp ${OUT_DATA}/graphics2/${graphic}.png)
  set(fr ${fr} ${OUT_DATA}/graphics2/${graphic}.png)
endforeach(graphic)

add_custom_target(resources
                  DEPENDS ${fr}
                 )
add_custom_target(resource_debug
                  COMMAND echo BITMAPS: ${BITMAPS}
                 )                  
