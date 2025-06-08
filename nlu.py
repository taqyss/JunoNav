import json
import re
from pathlib import Path


def load_json_strip(path: str):
    text = Path(path).read_text()
    cleaned = "\n".join(
        line
        for line in text.splitlines()
        if not line.strip().startswith("```")
    )
    return json.loads(cleaned)


def build_aliases(buildings):
    aliases = {}
    for b in buildings:
        names = {b["building_id"].lower(), b["building_name"].lower()}
        # also add simple forms removing punctuation
        names.add(re.sub(r"[^a-z0-9 ]+", "", b["building_name"].lower()))
        # add underscore form to match bus stop ids
        names.add(
            re.sub(r"[^a-z0-9]+", "_", b["building_name"].lower()).strip("_")
        )
        # remove text in parentheses for shorter alias
        base = re.sub(r"\s*\([^)]*\)", "", b["building_name"]).strip().lower()
        if base != b["building_name"].lower():
            names.add(base)
            names.add(re.sub(r"[^a-z0-9]+", "_", base).strip("_"))
        m = re.search(r"\(([^)]*)\)", b["building_name"])
        if m:
            inside = m.group(1).strip().lower()
            names.add(inside)
            names.add(re.sub(r"[^a-z0-9]+", "_", inside).strip("_"))
        for name in names:
            aliases[name] = b
    return aliases


def find_building(text, aliases):
    t = text.lower()
    for name, b in aliases.items():
        if name in t:
            return b
    return None


def classify_intent(text, buildings, bus_routes, aliases):
    t = text.lower()
    building = find_building(text, aliases)
    if building and any(w in t for w in ["open", "close", "hour"]):
        return {
            "intent": "opening_hours",
            "building_id": building["building_id"],
        }

    if any(w in t for w in ["shuttle", "bus", "route"]):
        if building:
            return {
                "intent": "shuttle_schedule",
                "destination": building["building_id"],
            }
        for r in bus_routes:
            if re.search(r"\b" + re.escape(r["route_id"].lower()) + r"\b", t):
                return {
                    "intent": "shuttle_schedule",
                    "route_id": r["route_id"],
                }
    if "malay" in t or "bahasa" in t:
        return {"intent": "language_switch", "language": "ms"}
    if "english" in t:
        return {"intent": "language_switch", "language": "en"}
    return {"intent": "unknown"}


def get_building_hours(building):
    open_t = building.get("opening_time", "")
    close_t = building.get("closing_time", "")
    if open_t and close_t:
        return (
            f"{building['building_name']} is open from {open_t} to {close_t}."
        )
    return f"I don't have opening hours for {building['building_name']}."


def get_shuttle_info(res, buildings, bus_routes, aliases):
    if "destination" in res:
        dest = aliases.get(res["destination"], None)
        if dest is None:
            # destination is already building_id
            dest_id = res["destination"]
            dest = next(
                (b for b in buildings if b["building_id"] == dest_id), None
            )
        dest_id = dest["building_id"] if dest else res["destination"]
        stop_keys = {dest_id}
        stop_keys.add(
            re.sub(r"[^a-z0-9]+", "_", dest["building_name"].lower()).strip(
                "_"
            )
        )
        m = re.search(r"\(([^)]*)\)", dest["building_name"])
        if m:
            stop_keys.add(
                re.sub(r"[^a-z0-9]+", "_", m.group(1).lower()).strip("_")
            )
        routes = [
            r for r in bus_routes if any(s in r["stops"] for s in stop_keys)
        ]
        if routes:
            route_names = ", ".join(r["route_id"] for r in routes)
            return (
                f"Routes {route_names} stop at {dest['building_name']}.",
                routes[0],
            )
        else:
            return f"No routes found for {dest['building_name']}.", None
    elif "route_id" in res:
        r = next(
            (r for r in bus_routes if r["route_id"] == res["route_id"]), None
        )
        if r:
            return f"{r['route_name']} stops: {', '.join(r['stops'])}.", r
        else:
            return "Route not found.", None
    return "Not enough info for shuttle.", None


def handle_query(text):
    buildings = load_json_strip("Buildings.json")
    bus_routes = load_json_strip("bus.json")
    aliases = build_aliases(buildings)
    res = classify_intent(text, buildings, bus_routes, aliases)
    if res["intent"] == "opening_hours":
        b = next(
            b for b in buildings if b["building_id"] == res["building_id"]
        )
        return get_building_hours(b)
    if res["intent"] == "shuttle_schedule":
        reply, _ = get_shuttle_info(res, buildings, bus_routes, aliases)
        return reply
    if res["intent"] == "language_switch":
        lang = res["language"]
        if lang == "ms":
            return "Baiklah, saya akan bercakap dalam Bahasa Melayu."
        return "Okay, I will continue in English."
    return "Sorry, I didn't understand that."


if __name__ == "__main__":
    import sys

    query = " ".join(sys.argv[1:])
    if not query:
        print("Usage: nlu.py <text>")
        sys.exit(1)
    print(handle_query(query))
