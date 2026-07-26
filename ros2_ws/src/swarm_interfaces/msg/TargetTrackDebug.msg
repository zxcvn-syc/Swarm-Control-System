# Debug-enhanced TargetTrack message for tracker_node diagnostics.
# Published on /target_track_debug alongside the primary /target_track topic.

std_msgs/Header header

# Array of tracks at this instant.  The base TargetTrack fields are
# copied verbatim so existing listeners (scheduler, enclosure) can continue
# to consume the main topic without change.
TargetTrack[] tracks

# --- Per-track debug data (parallel arrays, same length as tracks) ---
# 9-element row-major covariance of the Kalman filter state (x, y, vx, vy, ax, ay, ...).
# Consumers can reshape to (3, 3) for inspection.  Empty if the KF has not
# been initialised for a given track yet.
float64[] kf_covariance

# Human-readable reason strings for each track's current motion_mode, e.g.
# ["stationary: speed below threshold", "fast: acceleration detected"].
string[] motion_mode_reasons

# Raw appearance matching score from the re-id model (DeepSORT only).
# -1.0 if appearance features are disabled or the model is absent.
float64[] appearance_scores

# Which input topic (or frame source) triggered this debug publication.
# Useful when multiple tracker_node instances run in parallel.
string source_topic
