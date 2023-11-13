import ifcopenshell as ifc
from ifcopenshell.util.selector import Selector

class Converter:
    
    def __init__(self, path) -> None:
       pass
        
    
    def ifc2dwg(self, inputPath, outputPath):
        # Open the IFC file
        ifc_file = IfcFile()
        ifc_file.read(inputPath)

        # Select the IFC entities you want to export (optional)
        selector = Selector()
        selector.include("*")

        # Create a DWG file and export the selected IFC entities
        dwg_file = ifc_file.create_entity_instance(IfcFile.WRITER, "DWG")
        dwg_file.set_output_file(outputPath)
        dwg_file.write(selector.build())

        # Close the IFC and DWG files
        ifc_file.close()
        dwg_file.close()