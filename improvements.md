# Improvements

## 1. Follow-me recovery implemented; tune on the car
- AUTOPILOT now searches in place toward the person's last known side, then
  sweeps back once. Any confirmed person returns the car to FOLLOW.
- Startup waits 10 seconds, scans once, then returns to IDLE if nobody appears.
- Failed recovery after following waits stationary for reappearance.
- Tune search speed and sweep durations with the current inference cadence.

## 2. Actions were neglected lately, going further with them or exclude? maybe a bit pointless imo

## 3. Test follow-me gains and search rotation speed/durations

## 4. demo
