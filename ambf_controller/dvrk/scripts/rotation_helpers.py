    # Helpers for rotation....
    def create_rotZ( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (c, -s, 0), (s, c, 0) , (0,0,1) ) )
        return R

    def create_rotY( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (c, 0, s), (0, 1, 0) , (-s, 0, c) ) )
        return R

    def create_rotX( self, angle_rads  ):
        c, s = np.cos(angle_rads), np.sin(angle_rads)
        R = np.array( ( (1, 0, 0), (0, c, -s) , (0, s, c) ) )
        return R
    # Helpers for rotation END