keyboard = robot.getKeyboard()
keyboard.enable(timestep)

mode = 'manual' 
#mode = 'autonomous'
if mode == 'manual':
    key = keyboard.getKey()
    while(keyboard.getKey() != -1): pass
    if key == keyboard.LEFT :
        vL = -MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.RIGHT:
        vL = MAX_SPEED
        vR = -MAX_SPEED
    elif key == keyboard.UP:
        vL = MAX_SPEED
        vR = MAX_SPEED
    elif key == keyboard.DOWN:
        vL = -MAX_SPEED
        vR = -MAX_SPEED
    elif key == ord(' '):
        vL = 0
        vR = 0
    elif key == ord('S'):
        #reference: numpy.save(file, arr, allow_pickle=True, fix_imports=True)
        # Part 1.4: Filter map and save to filesystem
        map_bool = map > 0.2 
        map_bool = np.multiply(map_bool,1)
        def rotate90Clockwise(A):
            N = len(A[0])
            for i in range(N // 2):
                for j in range(i, N - i - 1):
                    temp = A[i][j]
                    A[i][j] = A[N - 1 - j][i]
                    A[N - 1 - j][i] = A[N - 1 - i][N - 1 - j]
                    A[N - 1 - i][N - 1 - j] = A[j][N - 1 - i]
                    A[j][N - 1 - i] = temp
        map_bool = rotate90Clockwise(map_bool)
        np.save('map.npy', map_bool)
        print("Map file saved")
    elif key == ord('L'):
        # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
        map = np.load('map.npy')
        print("Map loaded")
    else: # slow down
        vL *= 0.75
        vR *= 0.75