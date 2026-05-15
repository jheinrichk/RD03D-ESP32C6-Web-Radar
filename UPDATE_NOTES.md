# Update notes

## Feature updates and improvements

- Added fixed-size RAM log behavior documentation: the system stores up to 48 log items, replaces the oldest non-pinned entry when full and drops new entries only if all stored entries are pinned trail items.
- Added filtered confirmed-target radar display.
- Updated the browser radar to a 9 meter, 120 degree forward sector.
- Added detection classification for pedestrian, vehicle and unknown moving targets.
- Added event-based detection history.
- Added saved trails for movement paths of 4 meters or more.
- Clear Log feature deletes all stored log entries and saved trail detections from RAM.
- Logging only records movements with at least 2 meters of tracked path length.
- Increased filtering against foliage, breeze and micro movement.
- Centered the radar display in a stable UI frame.
- Added manual Reset Radar and automatic stall / frozen-content recovery.

## Commit message

```text
Improve RD03D filtered radar display, logging and UI
```
