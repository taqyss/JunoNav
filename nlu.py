import json
import re
from pathlib import Path

from sympy import false

#global language setting
#english is EN, malay is MS
language = "EN"
lastEnglishQuery = ""
lastBahasaQuery = ""


#load Json into Python dictionary
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
    if building and any(w in t for w in ["open", "close", "hour","buka", "tutup", "jam"]):
        return {
            "intent": "opening_hours",
            "building_id": building["building_id"],
        }

    if any(w in t for w in ["shuttle", "bus", "route","perjalanan ulang-alik", "bas", "laluan"]):
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
    return {"intent": "unknown"}


def get_building_hours(building):
    global language
    open_t = building.get("opening_time", "")
    close_t = building.get("closing_time", "")
    if language == "MS":
        #Malay version
        if open_t and close_t:
            return (
                f"{building['building_name']} buka dari pukul {open_t} hingga {close_t}."
            )
        return f"Saya tidak ada waktu operasi untuk {building['building_name']}."

    elif language == "EN":
        #English version
        if open_t and close_t:
            return (
                f"{building['building_name']} is open from {open_t} to {close_t}."
            )
        return f"I don't have opening hours for {building['building_name']}."


def get_shuttle_info(res, buildings, bus_routes, aliases):
    global language
    if language == "MS":
        #Malay version

        # Malay version
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
                    f"Laluan {route_names} berhenti di {dest['building_name']}.",
                    routes[0],
                )
            else:
                return f"Tiada laluan yang berhenti di {dest['building_name']}.",None

    elif language == "EN":
        #English version
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

def handle_query(englishQuery,bahasaQuery):
    #lower case both query
    englishQuery = englishQuery.lower()
    bahasaQuery = bahasaQuery.lower()

    #If non of the query don't have the word "juno", do nothing
    if not wake_word_detection(englishQuery) and not wake_word_detection(bahasaQuery):
        return

    global lastEnglishQuery, lastBahasaQuery
    #Ignore the query if it is exactly same as last one
    #The topic will keep publishing the last information if there are no new one.
    #Replace the remembered query if it recieved a new one
    if englishQuery == lastEnglishQuery and bahasaQuery == lastBahasaQuery:
        return
    else:
        lastEnglishQuery = englishQuery
        lastBahasaQuery = bahasaQuery

    #language switching detection
    if language_switching_detection(englishQuery,bahasaQuery):
        if language == "EN":
            return "Switch to English"
        elif language == "MS":
            return "Tukar ke Bahasa Malaysia"

    global buildings, bus_routes, text

    #Based on the language setting read the corresponding file
    if language == "MS":
        text = bahasaQuery
        buildings = load_json_strip("BuildingsMS.json")
        bus_routes = load_json_strip("BusMS.json")
    # If no matching, the default language is english
    else:
        text = englishQuery
        buildings = load_json_strip("BuildingsEN.json")
        bus_routes = load_json_strip("BusEN.json")
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
    if language == "MS":
        return "Maaf, saya tidak faham itu."
    elif language == "EN":
        return "Sorry, I didn't understand that."

#check juno in sentence or not
def wake_word_detection(text):
    return "juno" in text.lower()

#don't handle the case both English and Malay is used, but assume user usually don't do that
def language_switching_detection(englishQuery,bahasaQuery):
    global language
    combined_query = f"{englishQuery} {bahasaQuery}".lower()
    if "malay" in combined_query or "bahasa" in combined_query:
        language = "MS"
        return True
    elif "english" in combined_query:
        language = "EN"
        return True
    return False

# This part will be completely replaced by ROS sub.
if __name__ == "__main__":
    import sys

    def get_user_input():
        """Helper function to get user input with language switching support"""
        print("\nEnter your query (format: 'English query/Malay query') or 'exit' to quit:")
        print("\nYou must enter the key word (juno) otherwise it will ignore you")
        print("\nYou can't use exact same sentence continuously as it will assume it is getting the duplicate message from the topic ")
        user_input = input().strip()
        if user_input.lower() == 'exit':
            return None
        return user_input

    print("NLU System started. Enter queries in format: 'English query/Malay query'")

    while True:
        # Get user input
        query = get_user_input()
        if query is None:
            print("Exiting...")
            break

        try:
            # Split the input into English and Malay parts
            parts = query.split('/', 1)
            if len(parts) == 1:
                # If only one part provided, use it for both languages
                englishQuery = parts[0]
                bahasaQuery = parts[0]
            else:
                englishQuery, bahasaQuery = parts

            # Process and display the result
            result = handle_query(englishQuery.strip(), bahasaQuery.strip())
            print(f"\nResult: {result}")

        except Exception as e:
            print(f"Error processing query: {e}")
            print("Please use format: 'English query/Malay query'")
