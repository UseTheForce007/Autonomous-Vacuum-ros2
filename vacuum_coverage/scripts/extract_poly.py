import cv2
import yaml
import numpy as np
import os

def extract_room_polygons(yaml_path, pgm_path):
    # 1. Load map metadata from the YAML file
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    
    resolution = map_metadata['resolution']
    # Origin format in YAML is typically [x, y, yaw]
    origin_x = map_metadata['origin'][0]
    origin_y = map_metadata['origin'][1]
    occupied_thresh = map_metadata.get('occupied_thresh', 0.65)
    free_thresh = map_metadata.get('free_thresh', 0.196)

    # 2. Load the PGM image via OpenCV
    # OpenCV reads PGM images natively as grayscale (0-255)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"Error: Could not load image at {pgm_path}")
        return

    # In ROS maps: 
    # Light pixels (usually > 250) are free space. 
    # Dark pixels (usually 0) are walls/obstacles.
    # Gray pixels (around 205) are unknown space.
    
    # Create a binary mask where free space is white (255) and everything else is black (0)
    # We invert the standard thresholding because OpenCV contouring looks for white shapes.
    _, free_space_mask = cv2.threshold(img, int(255 * (1 - free_thresh)), 255, cv2.THRESH_BINARY)

    # 3. Morphological Operations to sever the doorway connection
    # Increase the kernel size if your rooms are still merging through the door!
    kernel_size = 9  
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    processed_mask = cv2.morphologyEx(free_space_mask, cv2.MORPH_OPEN, kernel)

    # 4. Find the contours of the rooms
    contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    rooms_polygons = []
    room_count = 1

    for contour in contours:
        # Ignore small artifacts (noise, door frames, or pillars)
        if cv2.contourArea(contour) < 500: 
            continue
        
        # Simplify the contour curves to straight geometric edges (polygons)
        epsilon = 0.015 * cv2.arcLength(contour, True)
        simplified_contour = cv2.approxPolyDP(contour, epsilon, True)
        
        world_points = []
        for point in simplified_contour:
            pixel_x = point[0][0]
            pixel_y = point[0][1]
            
            # CRITICAL MAP TRANSFORM: 
            # ROS maps store the image origin at the bottom-left corner, 
            # but OpenCV uses top-left as (0,0). We must flip the Y-axis.
            img_height = img.shape[0]
            flipped_pixel_y = img_height - pixel_y
            
            world_x = origin_x + (pixel_x * resolution)
            world_y = origin_y + (flipped_pixel_y * resolution)
            
            world_points.append({"x": float(world_x), "y": float(world_y)})
        
        rooms_polygons.append({f"room_{room_count}": world_points})
        print(f"Extracted Room {room_count} with {len(world_points)} vertices.")
        room_count += 1

    # 5. Save the coordinates out to a clean YAML file for Nav2 / OpenNav
    output_path = os.path.join(os.path.dirname(yaml_path), 'room_polygons.yaml')
    with open(output_path, 'w') as f:
        yaml.dump({"rooms": rooms_polygons}, f, default_flow_style=False)
    print(f"Successfully saved polygons to {output_path}")

if __name__ == '__main__':
    import sys
    if len(sys.argv) == 3:
        extract_room_polygons(sys.argv[1], sys.argv[2])
    else:
        extract_room_polygons('map_2.0.yaml', 'map_2.0.pgm')
