import pcbnew

# Define the desired width and height for all silkscreen text
TEXT_WIDTH = pcbnew.FromMM(0.8)
TEXT_HEIGHT = pcbnew.FromMM(0.8)

# Get the board object
board = pcbnew.GetBoard()

# Loop through all text items on the board
for drawing in board.GetDrawings():
    # Check if the drawing is a text item (generally indicated by having SetText() or GetText())
    if hasattr(drawing, 'SetTextWidth') and hasattr(drawing, 'SetTextHeight'):
        layer = drawing.GetLayer()
        if layer in (pcbnew.F_SilkS, pcbnew.B_SilkS):  # Front or Back silkscreen layers
            drawing.SetTextWidth(TEXT_WIDTH)
            drawing.SetTextHeight(TEXT_HEIGHT)

# Refresh the board view to show the changes
pcbnew.Refresh()

print("Silkscreen text sizes updated to 0.8mm x 0.8mm.")
