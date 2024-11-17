import os
from pdf2image import convert_from_path

def convert_pdf_to_png(pdf_path, output_dir, dpi=300):
    """
    Convert a PDF file to high resolution PNG images.
    
    Args:
        pdf_path (str): Path to the PDF file
        output_dir (str): Directory to save the PNG files
        dpi (int): Resolution of output images (default: 300)
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Convert PDF to images
        print(f"\nConverting PDF: {pdf_path}")
        print(f"Output directory: {output_dir}")
        print(f"Resolution: {dpi} DPI")
        print("\nProcessing...")
        
        images = convert_from_path(
            pdf_path,
            dpi=dpi,
            fmt='png',
            thread_count=os.cpu_count()  # Use all available CPU cores
        )
        
        # Save each page as a PNG file
        for i, image in enumerate(images):
            output_path = os.path.join(
                output_dir,
                f"{os.path.splitext(os.path.basename(pdf_path))[0]}_page_{i+1}.png"
            )
            image.save(output_path, 'PNG')
            print(f"✓ Saved page {i+1} to {output_path}")
            
    except Exception as e:
        print(f"\nError converting PDF: {str(e)}")
        return False
        
    return True

# Direct execution code
if __name__ == "__main__":
    # Get PDF path from user
    pdf_path = input("Enter the path to your PDF file: ").strip()
    
    # Check if file exists
    if not os.path.exists(pdf_path):
        print(f"Error: File '{pdf_path}' does not exist!")
        exit(1)
    
    # Get output directory
    output_dir = input("Enter output directory (press Enter for 'output'): ").strip()
    if not output_dir:
        output_dir = "output"
    
    # Get DPI
    try:
        dpi_input = 300
        dpi = int(dpi_input) if dpi_input else 300
    except ValueError:
        print("Invalid DPI value. Using default: 300")
        dpi = 300
    
    # Convert the PDF
    success = convert_pdf_to_png(pdf_path, output_dir, dpi)
    
    if success:
        print(f"\n Successfully converted {pdf_path} to PNG images!")
        print(f" Check the '{output_dir}' folder for your images.")
    else:
        print("\n Conversion failed!")