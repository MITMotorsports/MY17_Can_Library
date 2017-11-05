import fsae_can_spec
import MY17_Can_Library
import ids

spec_path = "ParseCAN/fsae_can_spec.yml"

fsae_can_spec.write("fsae_can_spec.h", spec_path)
MY17_Can_Library.write("../MY17_Can_Library.c", spec_path, "../MY17_Can_Library_BASE.txt", "special_cases.txt")
ids.write("../ids.h", spec_path)
