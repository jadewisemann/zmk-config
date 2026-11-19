from pathlib import Path
import re


config = Path(__file__).resolve().parents[1] / "config" / "keymap"
keymap = (config / "base.keymap").read_text()
behaviors = (config / "custom-behaviors.dtsi").read_text()
combos = (config / "combos.dtsi").read_text()


def keys(layer):
    body = keymap.split(f"LAYER({layer},", 1)[1].split("\n)\n", 1)[0]
    return re.findall(r"&kp\s+([A-Z]+)", body)


# One layout for both languages; the Korean 2-set arrangement is the reference.
assert keys("Base") == "Q W E R T Y U I O SLASH A S D F G H J K L P Z X C V B B N M COMMA DOT".split()
assert "LAYER(Korean," not in keymap and "KOREAN" not in keymap + combos + behaviors, "no separate Korean layer"
assert "bindings = <&mo>, <&kp>;" in behaviors
assert "&lang_num NUM LANG1" in keymap
assert "ZMK_COMBO(outer_thumb, MO_CAPS_SETTING,  31 32," in combos, "outer_thumb is the only way into Settings"
base_thumbs = keymap.split("LAYER(Base,", 1)[1].split("\n)\n", 1)[0]
assert "&sk LGUI" in base_thumbs and "&lang_num NUM F18" in base_thumbs, "macOS must be the default"
assert "#define TOGGLE_WIN &tog WIN_TOGGLE" in keymap
assert "MAC_" not in keymap + combos + behaviors
print("keymap check passed")
