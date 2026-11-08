"""Real PostgreSQL for the storage tests, or skip.

Order: ``$LOCATEBOT_TEST_PG_DSN`` (any reachable server; each test gets its own database), the
``pgserver`` package (bundled server, no root), or the system ``initdb``/``pg_ctl`` (Debian/Ubuntu
keep them in /usr/lib/postgresql/<v>/bin; runs as the current user in a temp dir). Otherwise the
tests that need a database are skipped; the recorder's logic tests use a fake and always run.
"""
import glob
import os
import shutil
import socket
import subprocess
import tempfile
import uuid

import pytest


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _system_bin(name: str) -> str | None:
    found = shutil.which(name)
    if found:
        return found
    hits = sorted(glob.glob(f"/usr/lib/postgresql/*/bin/{name}"))
    return hits[-1] if hits else None


@pytest.fixture(scope="session")
def pg_admin_dsn():
    pytest.importorskip("psycopg", reason="needs the db extra (psycopg)")
    dsn = os.environ.get("LOCATEBOT_TEST_PG_DSN")
    if dsn:
        yield dsn
        return
    try:
        import pgserver  # type: ignore
    except ImportError:
        pgserver = None
    if pgserver is not None:
        d = tempfile.mkdtemp(prefix="locatebot-pg-")
        srv = pgserver.get_server(d, cleanup_mode="stop")
        try:
            yield srv.get_uri()
        finally:
            srv.cleanup()
            shutil.rmtree(d, ignore_errors=True)
        return
    initdb, pg_ctl = _system_bin("initdb"), _system_bin("pg_ctl")
    if not (initdb and pg_ctl):
        pytest.skip("no PostgreSQL: set LOCATEBOT_TEST_PG_DSN, install pgserver, or install postgresql")
    d = tempfile.mkdtemp(prefix="locatebot-pg-")
    port = _free_port()
    subprocess.run([initdb, "-D", f"{d}/data", "-U", "postgres", "--auth=trust", "-E", "UTF8"],
                   check=True, capture_output=True)
    subprocess.run([pg_ctl, "-D", f"{d}/data", "-o", f"-p {port} -k {d} -c listen_addresses=''",
                    "-l", f"{d}/log", "-w", "start"], check=True, capture_output=True)
    try:
        yield f"postgresql://postgres@/postgres?host={d}&port={port}"
    finally:
        subprocess.run([pg_ctl, "-D", f"{d}/data", "-m", "immediate", "stop"], capture_output=True)
        shutil.rmtree(d, ignore_errors=True)


def _with_db(dsn: str, name: str) -> str:
    from psycopg.conninfo import conninfo_to_dict, make_conninfo
    params = conninfo_to_dict(dsn)
    params["dbname"] = name
    return make_conninfo(**params)


@pytest.fixture
def pg_dsn(pg_admin_dsn):
    """A fresh, empty database for one test."""
    import psycopg
    name = "t_" + uuid.uuid4().hex[:12]
    with psycopg.connect(pg_admin_dsn, autocommit=True) as c:
        c.execute(f'CREATE DATABASE "{name}"')
    try:
        yield _with_db(pg_admin_dsn, name)
    finally:
        with psycopg.connect(pg_admin_dsn, autocommit=True) as c:
            c.execute(f'DROP DATABASE IF EXISTS "{name}" WITH (FORCE)')
