# Regenerating the circuit diagram

Run from this repository's root. Requires Yosys and
[netlistsvg](https://github.com/nturley/netlistsvg) (`npm install -g netlistsvg`).

```bash
mkdir -p docs/images
yosys -p "
read_verilog rtl/core_axi_adapter.v
hierarchy -top core_axi_adapter
proc
opt_clean
write_json docs/images/core_axi_adapter.json
"
netlistsvg docs/images/core_axi_adapter.json -o docs/images/core_axi_adapter.svg
```

This module is a single FSM shared by both the read and write paths
(91 cells after `opt_clean`, already the minimum), so unlike most of the
other components it is diagrammed whole rather than split into
sub-blocks — a read/write split and a control/datapath split were both
tried and each came back at 70-73 cells out of 91, since the shared
state register touches nearly every output.

White-background fix, if you regenerate from scratch:

```bash
python3 -c "
import re
f = 'docs/images/core_axi_adapter.svg'
c = open(f).read()
m = re.search(r'(<svg\b[^>]*>)', c)
open(f, 'w').write(c[:m.end()] + '\n  <rect x=\"0\" y=\"0\" width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>' + c[m.end():])
"
```
