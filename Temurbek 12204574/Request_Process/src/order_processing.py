inventory = {
    'burger': 4,
    'pizza': 10,
    'kebab': 8,
    'spaghetti': 6,
    'mandi': 7,
    'burrito': 8,
}
print(f"Remaining Inventory: {inventory}")

def process_order(items):
    global inventory

    # Checking inventory
    available_items = check_inventory(items)

    if available_items:
        prepare_orders(available_items)
        return f"Order processed successfully.\nRemaining Inventory: {inventory}"
    else:
        return "Items not available"

def check_inventory(items):
    available_items = []

    global inventory  

    for item, quantity in items:
        if item in inventory and inventory[item] >= quantity:
            available_items.extend([item] * quantity)

    return available_items

def prepare_orders(items):
    # Simulated preparation process
    global inventory 

    for item in items:
        print(f"Preparing {item}...")
        inventory[item] -= 1  # Reducing the quantity after preparing an item

# Sample usage:
if __name__ == "__main__":
    items_to_order = [('pizza', 2)]  
    result = process_order(items_to_order)
    print(result)
