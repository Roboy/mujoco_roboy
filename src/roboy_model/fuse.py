import xml.etree.ElementTree as ET
import os.path

#####
# User inputs for model creation

while True:
    modelfile = input('Name of model (Default: model.xml -> Enter) ')
    if modelfile == '':
        modelfile = 'model.xml'
    if os.path.isfile(modelfile):
        break
    else:
        print('%s not found'%modelfile)

while True:
    cardsflowfile = input('Name of cardsflow (Default: cardsflow.xml -> Enter) ')
    if cardsflowfile == '':
        cardsflowfile = 'cardsflow.xml'
    if os.path.isfile(modelfile):
        break
    else:
        print('%s not found'%cardsflowfile)

outputfile = input('Name of output (Default: model_fused.xml -> Enter) ')
if outputfile == '':
    outputfile = 'model_fused.xml'

defaultsitecolor = '1 0.7 0 0.2'
sitecolor = input('Color for Sites. (Default: %s) '%defaultsitecolor)
if sitecolor == '':
    sitecolor = defaultsitecolor

defaulttendoncolor = '0.1 0.4 0.6 1'
tendoncolor = input('Color for Tendons. (Default: %s) '%defaulttendoncolor)
if tendoncolor == '':
    tendoncolor = defaulttendoncolor

modeltree = ET.parse(modelfile)
cardstree = ET.parse(cardsflowfile)

modelroot = modeltree.getroot()
cardsroot = cardstree.getroot()

# add site and tendon default color
default = modeltree.find('default')
if not default:
    default = ET.Element('default')
    modelroot.append(default)
defaultsite = default.find('site')
if defaultsite == None:
    defaultsite = ET.Element('site')
    default.append(defaultsite)
defaultsite.set('rgba',sitecolor)
defaulttendon = default.find('tendon')
if defaulttendon == None:
    defaulttendon = ET.Element('tendon')
    default.append(defaulttendon)
defaulttendon.set('rgba',tendoncolor)



####
# actual model creation

# iter (findall) over all myoMuscles of cardstree
for myoMuscle in cardstree.iter('myoMuscle'):
    # name of Muscle from cardsflow for tendon and actuator 
    muscleName = myoMuscle.get('name')
    # tendon location model
    tendon = modeltree.find('tendon')
    #actuator location model
    actuator = modeltree.find('actuator')
    # new spatial for each myoMuscle
    spatial = ET.Element('spatial')
    spatial.set('name',muscleName)
    tendon.append(spatial)
    # new muscle for each myoMuscle
    muscle = ET.Element('muscle')
    muscle.set('name', muscleName)
    muscle.set('tendon', muscleName)
    actuator.append(muscle)

    # find all links for each muscle
    for link in myoMuscle.findall('link'):
        linkname=link.get('name')
        # find the body of modeltree which has the link
        body=modelroot.find('.//body[@name="%s"]' % linkname)
        # some bodys do not exist, e.g. "torso"
        if body:
            # correct Site position is geomPosition + viaPointPosition
            # geomPosition = [*map(float, geomPosition)] changes text list in float list
            geomPosition = [*map(float, body.find('geom').get('pos').split())]
            # go through all viaPoints of the link and enumerate
            for count, viaPoint in enumerate(link.getchildren(), start=1):
                viaPointPosition = [*map(float, viaPoint.text.split())]
                #if more than one viaPoint, the sitenames are enumerated
                if len(link.getchildren()) > 1:
                    sitename=myoMuscle.get('name')+'_'+link.get('name')+'_'+str(count)
                else:
                    sitename=myoMuscle.get('name')+'_'+link.get('name')

                site = ET.Element('site')
                sitePosition = [a + b for a,b in zip(geomPosition, viaPointPosition)]
                sitePositionString = ' '.join(str(e) for e in sitePosition)
                site.set('name',sitename)
                site.set('pos',sitePositionString)
                body.append(site)

                site_tendon = ET.Element('site')
                site_tendon.set('site', sitename)
                spatial.append(site_tendon)

        else:
            print('ERROR\nBody of cardsflow.xml not found in model.xml!')

modeltree.write(outputfile)