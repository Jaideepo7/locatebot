"""PostgreSQL schema for the robot's shared storage, as numbered migrations.

Every table hangs off a ``robot_session`` (one run of ``tools/orin_run.py``), identified by a
uid generated on the robot, so writes recorded while the database was unreachable can be
replayed later in order without needing database-generated ids first.

Images live in the database (``image.data``, bytea), deduplicated by SHA-256. They are produced
on the Orin while the database runs on the Pi, so keeping them in Postgres avoids a second file
transfer path; a size cap (``RobotDB.prune``) keeps the table bounded and never deletes an image
an item still points to or one marked ``keep``.

SQL is plain PostgreSQL 15+ (tested on 16; the Pi 5's Debian trixie ships 17).
"""

MIGRATIONS: list[tuple[int, str]] = [
    (1, """
CREATE TABLE robot_session (
    uid         text PRIMARY KEY,
    started_at  timestamptz NOT NULL DEFAULT now(),
    host        text NOT NULL DEFAULT '',
    map_name    text NOT NULL DEFAULT '',
    meta        jsonb NOT NULL DEFAULT '{}'::jsonb
);

CREATE TABLE image (
    id          bigserial PRIMARY KEY,
    sha256      bytea NOT NULL UNIQUE,
    mime        text NOT NULL,
    width       integer NOT NULL,
    height      integer NOT NULL,
    nbytes      integer NOT NULL,
    data        bytea NOT NULL,
    kind        text NOT NULL CHECK (kind IN ('sighting', 'event', 'snapshot', 'verify', 'map')),
    session_uid text REFERENCES robot_session(uid) ON DELETE SET NULL,
    captured_s  double precision,
    keep        boolean NOT NULL DEFAULT false,
    created_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX image_created_idx ON image (created_at);

CREATE TABLE sighting (
    id          bigserial PRIMARY KEY,
    session_uid text NOT NULL REFERENCES robot_session(uid) ON DELETE CASCADE,
    item        text NOT NULL,
    x           double precision NOT NULL,
    y           double precision NOT NULL,
    sigma_m     double precision NOT NULL,
    range_m     double precision NOT NULL,
    bearing_rad double precision NOT NULL,
    captured_s  double precision NOT NULL,
    box         jsonb,
    image_id    bigint REFERENCES image(id) ON DELETE SET NULL,
    created_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX sighting_item_idx ON sighting (session_uid, item, id DESC);

CREATE TABLE item (
    session_uid   text NOT NULL REFERENCES robot_session(uid) ON DELETE CASCADE,
    name          text NOT NULL,
    x             double precision NOT NULL,
    y             double precision NOT NULL,
    sigma_m       double precision NOT NULL,
    sightings     integer NOT NULL,
    confirmed     boolean NOT NULL,
    last_seen_s   double precision NOT NULL,
    best_image_id bigint REFERENCES image(id) ON DELETE SET NULL,
    first_seen_at timestamptz NOT NULL DEFAULT now(),
    updated_at    timestamptz NOT NULL DEFAULT now(),
    PRIMARY KEY (session_uid, name)
);

CREATE TABLE event (
    id          bigserial PRIMARY KEY,
    session_uid text NOT NULL REFERENCES robot_session(uid) ON DELETE CASCADE,
    t_s         double precision NOT NULL,
    text        text NOT NULL,
    created_at  timestamptz NOT NULL DEFAULT now()
);
CREATE INDEX event_session_idx ON event (session_uid, id DESC);

CREATE TABLE mission (
    mission_id  text PRIMARY KEY,
    session_uid text REFERENCES robot_session(uid) ON DELETE SET NULL,
    state       text NOT NULL,
    message     text NOT NULL DEFAULT '',
    reason      text NOT NULL DEFAULT '',
    created_at  timestamptz NOT NULL DEFAULT now(),
    updated_at  timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE mission_status (
    id          bigserial PRIMARY KEY,
    mission_id  text NOT NULL REFERENCES mission(mission_id) ON DELETE CASCADE,
    state       text NOT NULL,
    message     text NOT NULL DEFAULT '',
    reason      text NOT NULL DEFAULT '',
    created_at  timestamptz NOT NULL DEFAULT now()
);

CREATE TABLE map_snapshot (
    id          bigserial PRIMARY KEY,
    session_uid text REFERENCES robot_session(uid) ON DELETE SET NULL,
    name        text NOT NULL,
    lbmap       bytea NOT NULL,
    image_id    bigint REFERENCES image(id) ON DELETE SET NULL,
    created_at  timestamptz NOT NULL DEFAULT now()
);
"""),
]
