"""ELF symbol lookup that understands file-local (static) symbols.

elf_symbols_v3(elf, names) returns {name: (addr, size)}. A name may be
written "file.c:var" to pick a static variable from one source file (the
symbol table lists each file's locals after an STT_FILE entry). Plain names
prefer the GLOBAL symbol.
"""
from elftools.elf.elffile import ELFFile


def elf_symbols_v3(path, names):
    want = set(names)
    out = {}
    with open(path, "rb") as f:
        for sec in ELFFile(f).iter_sections():
            if sec.header.sh_type != "SHT_SYMTAB":
                continue
            cur_file = None
            for s in sec.iter_symbols():
                typ = s.entry.st_info.type
                bind = s.entry.st_info.bind
                if typ == "STT_FILE":
                    cur_file = s.name
                    continue
                val = (int(s.entry.st_value), int(s.entry.st_size))
                q = "%s:%s" % (cur_file, s.name)
                if bind == "STB_LOCAL" and q in want:
                    out[q] = val
                if s.name in want and (bind == "STB_GLOBAL" or s.name not in out):
                    out[s.name] = val
    return out
