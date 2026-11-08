import pathlib

import pytest

from locatebot.backend.commands import Kind, parse_command


def test_find_with_description():  # P1
    c = parse_command("/find blue toolbox")
    assert (c.kind, c.description, c.error) == (Kind.FIND, "blue toolbox", None)


def test_find_whitespace_normalised():  # P2
    assert parse_command("/find   Blue   Toolbox  ").description == "Blue Toolbox"


@pytest.mark.parametrize("text", ["/find", "/find    ", "/find ???", "/find 🔧"])
def test_find_needs_description(text):  # P3, P10
    c = parse_command(text)
    assert c.kind == Kind.FIND and c.description is None and c.error


def test_group_form_and_case():  # P4, P5
    assert parse_command("/find@MyRobotBot red cup").description == "red cup"
    assert parse_command("/FIND red cup").description == "red cup"
    assert parse_command("/Status@MyRobotBot").kind == Kind.STATUS


@pytest.mark.parametrize("text,kind", [("/start", Kind.START), ("/help", Kind.HELP), ("/status", Kind.STATUS),
                                       ("/cancel", Kind.CANCEL), ("/cancel now please", Kind.CANCEL),
                                       ("  /help  ", Kind.HELP)])
def test_simple_commands(text, kind):  # P6
    assert parse_command(text).kind == kind


def test_plain_text_is_not_a_mission():  # P7
    assert parse_command("blue toolbox").kind == Kind.NOT_COMMAND


def test_unknown_command():  # P8
    assert parse_command("/dance").kind == Kind.UNKNOWN


def test_too_long_rejected():  # P9
    c = parse_command("/find " + "x" * 200)
    assert c.description is None and "too long" in c.error


@pytest.mark.parametrize("text", ["/find $(rm -rf /)", "/find ; reboot", "/find a`id`b", "/find cup\x00"])
def test_shell_like_input_rejected(text):  # P11, P13
    c = parse_command(text)
    assert c.description is None and c.error


def test_motion_words_are_just_a_description():  # P12
    assert parse_command("/find forward 5m full speed").description == "forward 5m full speed"


def test_newline_does_not_survive():  # P13
    c = parse_command("/find red\ncup")
    assert c.description == "red cup"


def test_non_text_content():  # P14
    assert parse_command(None).kind == Kind.NOT_TEXT


def test_no_shell_execution_reachable():  # S2
    root = pathlib.Path(__file__).resolve().parents[2] / "locatebot"
    for pkg in ("backend", "dispatch"):
        for f in (root / pkg).rglob("*.py"):
            src = f.read_text()
            for bad in ("subprocess", "os.system", "os.popen", "eval(", "exec(", "pickle"):
                assert bad not in src, f"{bad} in {f}"
