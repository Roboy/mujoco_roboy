import xml.etree.ElementTree as ET
modeltree = ET.parse('model.xml')
modeltree.write("model_output.xml")