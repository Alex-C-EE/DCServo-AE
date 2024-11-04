import pcbnew

def enable_teardrops_for_all_vias():
    """
    Enables teardrops for all vias in the current PCB project.
    """
    # Access the current board
    board = pcbnew.GetBoard()
    if not board:
        print("Error: No board loaded.")
        return

    # Counters for reporting
    via_count = 0
    teardrop_enabled_count = 0

    # Check if the board object has a GetVias() method
    if not hasattr(board, 'GetVias'):
        print("Error: The board object does not have a 'GetVias' method. Please verify the KiCad 8 API.")
        return

    # Iterate through all vias on the board
    for via in board.GetVias():
        via_count += 1
        try:
            # Access the teardrop properties
            teardrop = via.GetTeardrop()

            if teardrop:
                # Check if teardrop is already enabled
                if not teardrop.IsEnabled():
                    teardrop.Enable()
                    # Optionally, set teardrop parameters like size and style
                    # Available options may include: SMALL, MEDIUM, LARGE for size
                    # and NONE, STRAIGHT, CURVED for style

                    # Set teardrop size
                    try:
                        teardrop.SetSize(pcbnew.TEARDROP_SIZE_MEDIUM)  # Options: SMALL, MEDIUM, LARGE
                    except AttributeError:
                        print("Warning: 'SetSize' method not found for teardrop. Skipping size setting.")

                    # Set teardrop style
                    try:
                        teardrop.SetStyle(pcbnew.TEARDROP_STYLE_CURVED)  # Options: NONE, STRAIGHT, CURVED
                    except AttributeError:
                        print("Warning: 'SetStyle' method not found for teardrop. Skipping style setting.")

                    teardrop_enabled_count += 1
                    # Get via position in millimeters for readability
                    pos_mm = pcbnew.ToMM(via.GetPosition())
                    print(f"Teardrop enabled for via at ({pos_mm.x:.2f} mm, {pos_mm.y:.2f} mm).")
            else:
                print(f"Warning: Via at {via.GetPosition()} does not have teardrop properties accessible.")
        except AttributeError as e:
            print(f"Error processing via at {via.GetPosition()}: {e}")

    # Commit changes to the board
    board.BuildHierarchy()
    pcbnew.Refresh()

    # Summary
    print("\nTeardrop Enhancement Summary:")
    print(f"Total vias found: {via_count}")
    print(f"Teardrops enabled: {teardrop_enabled_count}")

# Execute the function
enable_teardrops_for_all_vias()
