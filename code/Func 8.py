   # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    dist, angles = to_polar_coords(xpix_navigable, ypix_navigable)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
        # Same for rock samples
    dist, angles = to_polar_coords(xpix_rocks, ypix_rocks)
    Rover.samples_dists = dist
    Rover.samples_angles = angles
