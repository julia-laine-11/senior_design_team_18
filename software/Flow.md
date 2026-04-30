graph TD
    A[Start Application] --> B[Initialize GameState & Load Config]
    B --> C[Start CameraReader Thread]
    B --> D[Start Tracking & Defense Thread]
    B --> E[Start ControlGUI Mainloop]

    subgraph Camera_Thread [Camera Thread (V4L2 Capture)]
        direction TB
        C1[Read V4L2 Frame] --> C2[Update Latest Frame Buffer]
        C2 -. Loop .-> C1
    end

    subgraph GUI_Thread [GUI Thread (Tkinter)]
        direction TB
        E1[Listen for Slider/Button Input] --> E2[Update Shared GameState]
        E2 --> E3[Refresh GUI Status Panels]
        E3 -. Loop .-> E1
    end

    subgraph Main_Tracking_Thread [Tracking & Defense Thread]
        direction TB
        D1[Get Latest Frame from Camera Thread] --> D2[Vision: Resize, Blur, HSV Masks]
        D2 --> D3[Find Contours & Identify Objects]
        D3 --> D4[Kalman Filter Tracking: Puck & Mallet]
        D4 --> D5[Fetch Current State: Keys, GUI Config, STM32 UART]
        D5 --> D6[[Execute Defense Logic Module]]
        D6 --> D7[Motor Control: Apply Acceleration Ramping]
        D7 --> D8[Command CoreXY Motor Controller]
        D8 --> D9[Draw OpenCV Visualization & Show Frame]
        D9 -. Loop .-> D1
    end