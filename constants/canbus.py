class kCANId:
    ROBORIO = 0
    PDH = 31
    CANDLE = 32

    # NOTE: These are not used, but referneces for Drive Subsystem
    class drive:
        FL_DRIVE = 1
        FR_DRIVE = 2
        BL_DRIVE = 3
        BR_DRIVE = 4
        FL_STEER = 5
        FR_STEER = 6
        BL_STEER = 7
        BR_STEER = 8
        FL_CANCODER = 9
        FR_CANCODER = 10
        BL_CANCODER = 11
        BR_CANCODER = 12
        PIGEON = 13

    class intake:
        SPINNER = 16
        LEFT_PIVOT = 17
        RIGHT_PIVOT = 18

    class hopper:
        SPINDEXER = 21
        TRASNFER = 22

    class shooter:
        BOTTOM_MOTOR = 26
        TOP_MOTOR = 27
        HOOD_CONTROL = 28
    
    class climb:
        MOTOR = 41
