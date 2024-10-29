import pcbnew

board = pcbnew.GetBoard()
zones = board.Zones()

for zone in zones:
    filled_area = zone.GetFilledArea() / 1e12  # Convert to square mm
    outline_area = zone.GetOutlineArea() / 1e12  # Convert to square mm
    
    print(f"Zone for {zone.GetNetname()}:")
    print(f"  Filled Area: {filled_area:.6f} mm²")
    print(f"  Outline Area: {outline_area:.6f} mm²")
