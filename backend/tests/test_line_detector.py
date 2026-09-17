from pinky_control_center.line_detector import detect_line
from PIL import Image
import io

def _jpeg(width_bar_x0, line="white", size=(320, 240)):
    bg = (40, 40, 40) if line == "white" else (220, 220, 220)
    fg = (255, 255, 255) if line == "white" else (0, 0, 0)
    img = Image.new("RGB", size, bg)
    px = img.load()
    for y in range(size[1] // 2, size[1]):
        for x in range(width_bar_x0, width_bar_x0 + 30):
            px[x, y] = fg
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    return buf.getvalue()

def test_white_bar_left_is_negative_offset():
    r = detect_line(_jpeg(20, "white"), "white")
    assert r.found is True and r.offset < -0.3 and r.polarity == "white"

def test_black_bar_right_is_positive_offset():
    r = detect_line(_jpeg(250, "black"), "black")
    assert r.found is True and r.offset > 0.3 and r.polarity == "black"

def test_auto_detects_both():
    assert detect_line(_jpeg(20, "white"), "auto").polarity == "white"
    assert detect_line(_jpeg(250, "black"), "auto").polarity == "black"

def test_auto_finds_black_line_on_dim_light_floor():
    img = Image.new("RGB", (320, 240), (100, 100, 100))
    px = img.load()
    for y in range(120, 240):
        for x in range(250, 280):
            px[x, y] = (10, 10, 10)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    r = detect_line(buf.getvalue(), "auto")
    assert r.found is True
    assert r.polarity == "black"
    assert r.offset > 0.3

def test_manual_mode_survives_dim_lighting():
    img = Image.new("RGB", (320, 240), (100, 100, 100))
    px = img.load()
    for y in range(120, 240):
        for x in range(20, 50):
            px[x, y] = (255, 255, 255)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    r = detect_line(buf.getvalue(), "white")
    assert r.found is True
    assert r.offset < -0.3

def test_empty_floor_is_loss():
    img = Image.new("RGB", (320, 240), (40, 40, 40))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    assert detect_line(buf.getvalue(), "auto").found is False

def test_empty_light_floor_is_loss():
    img = Image.new("RGB", (320, 240), (220, 220, 220))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=90)
    assert detect_line(buf.getvalue(), "auto").found is False
