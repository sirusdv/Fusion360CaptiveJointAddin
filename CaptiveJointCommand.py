import adsk.core
import adsk.fusion
import traceback
from .Fusion360Utilities import Fusion360Utilities as futil
from .Fusion360Utilities.Fusion360CommandBase import Fusion360CommandBase


def alert(s):
     futil.get_app_objects()['ui'].messageBox(s)

def cast_ray_inverse_normal(from_component, from_face, from_point):
    # Get normal vector to face in opposite direction
    (normal_return, normal_vector) = from_face.evaluator.getNormalAtPoint(from_point)
    normal_vector.scaleBy(-1.0)

    # Cast ray to determine next face
    hit_faces = from_component.findBRepUsingRay(from_point, normal_vector, 1)

    # Check if source face is included in returned set, function of ray cast tolerance
    if hit_faces[0].tempId == from_face.tempId:
        next_face = hit_faces[1]
    else:
        next_face = hit_faces[0]

    return next_face, normal_vector


# Creates the equivalent of a to next extrude
def to_next_extrude(profiles_, target_component, target_face, center_point_sketch, operation):
  
    next_face, _ = cast_ray_inverse_normal(target_component, target_face, center_point_sketch)        

    # Create an extrusion input to be able to define the input needed for an extrusion
    extrudes = target_component.features.extrudeFeatures
    ext_input = extrudes.createInput(profiles_, operation)
    to_next_extent = adsk.fusion.ToEntityExtentDefinition.create(next_face, False)
    ext_input.setOneSideExtent(to_next_extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)

    try:
        extrude_feature = extrudes.add(ext_input)
        return extrude_feature

    except:
        raise Exception('It appears your top hole is not completely terminated by the opposite face')


def create_top_sketch(point, radius):
    target_component = point.parentSketch.parentComponent

    # Get point in world CS
    world_point = point.worldGeometry

    # Create a new sketch on the plane.
    sketches = target_component.sketches

    # Get target face for sketch
    target_face = target_component.findBRepUsingPoint(world_point, 1)[0]


    try:
        sketch = sketches.add(target_face)

    except:
        raise Exception('The point you selected does not lie on a valid face')

    center_point_sketch = sketch.project(point)[0]
    holeCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(center_point_sketch, radius)
    return sketch, center_point_sketch, target_component, target_face



def create_top_hole(point, config):
    sketch, center_point_sketch, target_component, target_face = create_top_sketch(point, config['bolt_radius'])

    
    hole_feature = to_next_extrude(sketch.profiles[sketch.profiles.count - 1], target_component, target_face, center_point_sketch.worldGeometry, adsk.fusion.FeatureOperations.CutFeatureOperation)

    #get the face that we would have hit if we continued to extrude
    next_face, _ = cast_ray_inverse_normal(target_component,target_face,center_point_sketch.worldGeometry)

    return next_face

def translate_point_by_vector(startPoint, magnitude, vector):

    newPoint = startPoint.copy()

    translationVector = vector.copy()
    translationVector.scaleBy(magnitude)
    newPoint.translateBy(translationVector)

    return newPoint
  

def create_side_sketch(next_face, centerPoint, config):
    
    target_face = None
    target_edge = None
    target_face_len = -1
    for edge in next_face.edges:

        neighbor_face = edge.faces.item(0)
        if neighbor_face == next_face:
            neighbor_face = edge.faces.item(1)


        if edge.length > target_face_len:
            target_edge = edge
            target_face = neighbor_face
            target_face_len = edge.length

    #create side sketch to xfer the center point to the edge
    side_sketch = next_face.body.parentComponent.sketches.add(next_face)
    side_proj_point = side_sketch.project(centerPoint)[0]
    side_sketch.isVisible = False

    #create main sketch
    new_sketch = target_face.body.parentComponent.sketches.add(target_face)

    #compute X vector
    proj_point = new_sketch.project(centerPoint)[0]
    startPoint = new_sketch.project(side_proj_point)[0]

    Xvector = proj_point.geometry.vectorTo(startPoint.geometry)
    Xvector.normalize()


    #alert("Xvector = " + str(Xvector.x) + "," + str(Xvector.y) + "," + str(Xvector.z))

    #compute Y vector from edge
    projected_edge = new_sketch.project(target_edge)[0]

    Yvector = projected_edge.startSketchPoint.geometry.vectorTo(projected_edge.endSketchPoint.geometry)
    Yvector.normalize()
    #alert("Yvector = " + str(Yvector.x) + "," + str(Yvector.y) + "," + str(Yvector.z))

    matThickness = proj_point.geometry.distanceTo(startPoint.geometry)

    endPoint = translate_point_by_vector(proj_point.geometry, config['bolt_length'] + config['extra_tail'], Xvector)

    #create our sketch mirror line.
    mirrorLine = new_sketch.sketchCurves.sketchLines.addByTwoPoints(proj_point, endPoint)
    mirrorLine.isConstruction = True

    #we're going to want to do this twice, once per Y direction

    for yscaler in [1.0, -1.0]:
        scopedYvector = Yvector.copy()
        scopedYvector.scaleBy(yscaler)

        #create barrel shaft leading up to nut
        shaftsegstart = translate_point_by_vector(startPoint.geometry,config['bolt_radius'],scopedYvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint.geometry, shaftsegstart)

        shaftsegend = translate_point_by_vector(shaftsegstart, config['bolt_length'] - matThickness - config['nut_depth'], Xvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(shaftsegstart, shaftsegend)

        #create nut hole shape
        nutoffset = (config['nut_width'] / 2.0) - config['bolt_radius']
        
        nuttop = translate_point_by_vector(shaftsegend, nutoffset, scopedYvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(shaftsegend, nuttop)

        nutend = translate_point_by_vector(nuttop, config['nut_depth'], Xvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(nuttop, nutend)

        nutbottom = translate_point_by_vector(nutend, -1.0 * config['nut_width']/2 + config['bolt_radius'] , scopedYvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(nutend, nutbottom)

        #create the extra tail for tensioning

        tail = translate_point_by_vector(nutbottom, config['extra_tail'], Xvector)
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(nutbottom, tail)

        #close finish the back
        new_sketch.sketchCurves.sketchLines.addByTwoPoints(tail, endPoint)

    return new_sketch, mirrorLine, target_face

def create_side_hole(next_face, centerPoint, config):
    sketch, mirrorLine, face = create_side_sketch(next_face, centerPoint, config)
    hole_feature = to_next_extrude(sketch.profiles[sketch.profiles.count - 1], face.body.parentComponent, face, face.pointOnFace, adsk.fusion.FeatureOperations.CutFeatureOperation)

# Delete the line that says "pass" for any method you want to use
class CaptiveJointCommand(Fusion360CommandBase):


    def on_preview(self, command, inputs, args, input_values):
        
        config = {
        "bolt_radius":  input_values['bolt_radius'],
        "bolt_length": input_values['bolt_length'],
        "nut_width": input_values['nut_width'],
        "nut_depth": input_values['nut_depth'],
        "extra_tail": input_values['extra_tail']
        }

        
        try:
            for holepoint in input_values['target_points']:        
                start = futil.start_group()
                next_face = create_top_hole(holepoint, config)
                create_side_hole(next_face, holepoint, config)
                futil.end_group(start)

            args.isValidResult = True
        except Exception as e:
            args.isValidResult = False
            alert("Invalid selection: \n\n" + str(e))
        pass

    def on_create(self, command, command_inputs):

        targetInput = command_inputs.addSelectionInput('target_points', 'Target Points', 'Select hole center-points')
        targetInput.addSelectionFilter('SketchPoints')
        targetInput.setSelectionLimits(1)

        command_inputs.addValueInput("bolt_radius", "Bolt Shaft Radius", "mm", adsk.core.ValueInput.createByString("(3mm)/2"))
        command_inputs.addValueInput("bolt_length", "Bolt Shaft Length", "mm", adsk.core.ValueInput.createByString("15mm"))
        command_inputs.addValueInput("nut_width", "Nut Width", "mm", adsk.core.ValueInput.createByString("5.4mm"))
        command_inputs.addValueInput("nut_depth", "Nut Depth", "mm", adsk.core.ValueInput.createByString("2.37mm"))
        command_inputs.addValueInput("extra_tail", "Extra Tail", "mm", adsk.core.ValueInput.createByString("3mm"))