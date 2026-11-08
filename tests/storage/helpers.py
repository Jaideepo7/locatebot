import struct


def jpeg(tag: bytes = b"x", w: int = 640, h: int = 360) -> bytes:
    """Smallest JPEG-shaped bytes our parser accepts (SOI, comment, SOF0, EOI); ``tag`` varies the hash."""
    com = b"locatebot-test:" + tag
    return (b"\xff\xd8" + b"\xff\xfe" + struct.pack(">H", len(com) + 2) + com
            + b"\xff\xc0" + struct.pack(">HBHHB", 8, 8, h, w, 3) + b"\xff\xd9")
