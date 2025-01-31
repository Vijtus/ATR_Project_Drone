# Custom drone controller - Project Schedule

## Tasks
 - Task 1: Perform initial research and select parts for the controller board.
 - Task 2: Assemble and verify the operation of the control board.
 - Task 3: Implement a simple HAL for the different drone components.
 - Task 4: Implement a draft of the researched control algorithm in a high level language and perform rudimentary simulation and evaluations of the chosen control scheme.
 - Task 5: Implement said algorithm in an efficient manner in C for the microcontroller onboard.
 - Task 6: Test the initial implementation and note down things that need improvements.
 - Repeat 4-6 and iterate until stable level flight is achieved.
 - Task 7: Compile all the knowledge and insights gained from the above and compile it into a final presentation.

## Milestones
 - Milestone no. 1: Assembling and testing a working control board.
 - Milestone no. 2: Being able to control: motor speeds, read control signals from the RC, read battery voltage and read the IMU thought the HAL.
 - Milestone no. 3: Having a working simulation of the PID loops and other control systems, being able to feed generated or measured sensor data into it to see how the system responds.
 - Milestone no. 4: Completing the first flight with our own controller.
 - Milestone no. 5: Completing a level flight with the final stabilization algorithm.
 - Milestone no. 6: [Final presentation]: Compiling documentation, code, photos and videos from the above and creating a final presentation.

 ## Gantt chart
```mermaid
gantt
    dateFormat  YYYY-MM-DD
    title Drone Development Project Gantt Chart

    section Research
    Review Drone Designs           :a1, 2023-10-24, 10d
    List Requirements              :a2, after a1, 8d
    Research Components            :a3, after a2, 8d
    
    section Controller Board Development
    Assemble Controller Board      :b1, 2023-11-13, 5d
    Test Controller Board          :b2, after b1, 5d
    
    section HAL Development
    Develop HAL                    :c1, 2023-11-25, 7d
    Test HAL                       :c2, after c1, 5d
    
    section Control Algorithm Development
    Implement Control Algorithm    :d1, 2023-12-05, 10d
    Simulate and Test              :d2, after d1, 10d
    
    section Onboard Implementation
    Implement on Microcontroller   :e1, after d2, 10d
    Optimize for Performance       :e2, after e1, 7d
    
    section Flight Testing
    Conduct Flight Tests           :f1, after e2, 10d
    Address Issues and Iterate     :f2, after f1, 7d
    
    section Documentation and Reporting
    Compile Project Documentation  :g1, after f2, 7d
    Create Final Presentation      :g2, after g1, 5d
    
    section Milestones
    Research Complete              :m1, after a3, 2023-11-13
    Controller Board Ready         :m2, after b2, 2023-11-20
    HAL Ready                      :m3, after c2, 2023-12-04
    Control Algorithm Draft Ready  :m4, after d1, 2023-12-15
    Simulation Testing Complete    :m5, after d2, 2023-12-25
    Onboard Implementation Ready   :m6, after e2, 2024-01-01
    Initial Flight Tests Complete  :m7, after f1, 2024-01-10
    Project Ready for Final Review :m8, after g1, 2024-01-17
    Final Presentation Ready       :m9, after g2, 2024-01-22
```

More about how to make a gantt chart in mermaid, you will find here: [Mermaid Gantt](https://mermaid.js.org/syntax/gantt.html)
