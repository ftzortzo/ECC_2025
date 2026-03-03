#!/usr/bin/env python3
import os, sys, re, argparse
from pathlib import Path

ORIGIN_RE = re.compile(r"^\s*Origin\s+(\d+)\s*$", re.IGNORECASE)
PAIR_RE   = re.compile(r"(\d+)\s*:\s*([0-9.eE+-]+)")
DEFAULT_ZONES = list(range(1, 25))  # 1..24

def require_sumo_tools():
    sumo_home = os.environ.get("SUMO_HOME")
    if not sumo_home:
        raise RuntimeError("SUMO_HOME is not set.")
    sys.path.append(str(Path(sumo_home) / "tools"))
    import sumolib  # noqa
    return sumo_home

def read_net(net_path):
    import sumolib
    return sumolib.net.readNet(str(net_path))

def pick_edges_for_node(net, node_id: str):
    n = net.getNode(node_id)
    if n is None:
        return [], []
    out_edges = [e.getID() for e in n.getOutgoing() if not e.getID().startswith(":")]
    in_edges  = [e.getID() for e in n.getIncoming() if not e.getID().startswith(":")]
    return out_edges, in_edges

def write_taz_xml(net, out_taz, zones, node_id_map=None, weight=1.0):
    node_id_map = node_id_map or {}
    out_taz = Path(out_taz)
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<tazs>"]
    for z in zones:
        zid = str(z)
        nid = node_id_map.get(zid, zid)
        src, snk = pick_edges_for_node(net, nid)
        lines.append(f'  <taz id="{zid}">')
        for e in src:
            lines.append(f'    <tazSource id="{e}" weight="{weight}"/>')
        for e in snk:
            lines.append(f'    <tazSink id="{e}" weight="{weight}"/>')
        lines.append("  </taz>")
    lines.append("</tazs>")
    out_taz.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {out_taz}")

def parse_tntp_trips(tntp_path):
    pairs = []
    cur_origin = None
    with open(tntp_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            m = ORIGIN_RE.match(line)
            if m:
                cur_origin = m.group(1)
                continue
            if cur_origin is None:
                continue
            for to_, val in PAIR_RE.findall(line):
                try:
                    cnt = float(val)
                except ValueError:
                    continue
                if cnt > 0:
                    pairs.append((cur_origin, to_, cnt))
    return pairs

def write_tazrelation_xml(out_path, pairs, begin=0, end=3600, interval_id="car"):
    out_path = Path(out_path)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<data xmlns:xsi="https://www.w3.org/2001/XMLSchema-instance" '
        'xsi:noNamespaceSchemaLocation="https://sumo.dlr.de/xsd/datamode_file.xsd">',
        f'  <interval id="{interval_id}" begin="{begin}" end="{end}">'
    ]
    for fr, to, cnt in pairs:
        lines.append(f'    <tazRelation from="{fr}" to="{to}" count="{cnt}"/>')
    lines += ["  </interval>", "</data>"]
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {out_path} with {len(pairs)} OD pairs")

def main():
    require_sumo_tools()
    ap = argparse.ArgumentParser()
    ap.add_argument("--net", default="siouxfalls.net.xml")
    ap.add_argument("--tntp", default="SiouxFalls_trips.tntp")
    ap.add_argument("--taz-out", default="siouxfalls.taz.xml")
    ap.add_argument("--rel-out", default="siouxfalls.tazRel.xml")
    ap.add_argument("--begin", type=float, default=0)
    ap.add_argument("--end", type=float, default=3600)
    ap.add_argument("--interval-id", default="car")
    args = ap.parse_args()

    net_path = Path(args.net)
    tntp_path = Path(args.tntp)
    if not net_path.exists():
        raise FileNotFoundError(net_path)
    if not tntp_path.exists():
        raise FileNotFoundError(tntp_path)

    net = read_net(net_path)
    NODE_ID_MAP = {}  # adjust only if zone-node ids differ

    write_taz_xml(net, args.taz_out, DEFAULT_ZONES, node_id_map=NODE_ID_MAP, weight=1.0)
    pairs = parse_tntp_trips(tntp_path)
    write_tazrelation_xml(args.rel_out, pairs, begin=args.begin, end=args.end, interval_id=args.interval_id)

    print("\nNext commands:")
    print(f'  od2trips -n "{args.taz_out}" -z "{args.rel_out}" -o "siouxfalls.trips.xml" --begin {args.begin} --end {args.end} --spread.uniform')
    print('  duarouter -n "siouxfalls.net.xml" -r "siouxfalls.trips.xml" -a "vtypes.add.xml" -o "siouxfalls.rou.xml"')

if __name__ == "__main__":
    main()