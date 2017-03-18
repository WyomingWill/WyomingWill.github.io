;;Dettagli sull'arena:
;;Le dimensioni sono di 2x2 m^2. Ogni patch rappresenta 1 cm. La grandezza della patch, per questioni
;;di visualizzazione, � stata fissata a 3 pixel.Come time-step, si prende 1 secondo.

;;Initialization of the model
extensions [array]

breed [scouts scout]

globals [
  GIR.radius    ;;standard proximity sensors detection radius
  dt ;;time variable
  previous_deployment ;;used to update the simulation when the Deployment is changed
]

turtles-own [
  ;; Hardware parameters
  IR.radius  ;;proximity sensors detection radius
  Fmax  ;;Needed to limit maximal acceleration
  Vmax  ;;Needed to limit maximal velocity
  vel ;;modulus of the velocity vector
  alpha ;;phase of the velocity vector
  mass ;;mass
  
  ;; Force parameters
  epsilon  
  d
  c
  
  ;; Other
  distanceX ;;distance with the other robots - x axes
  distanceY ;;distance with the other robots - y axes
  posX ;;distance from the barycenter of the deployment (used in the Tree and the Star deployment) - x axes
  posY ;;distance from the barycenter of the deployment (used in the Tree and the Star deployment) - y axes
  interArray ;;array of booleans identifying whether there is or not an interaction with the ?-th robot
  interLevel ;;containing the lower level of the neighbor robots
  force ;;force
  deltaX ;;distance of the next movement on the x axes
  deltaY ;;distance of the next movement on the y axes
  interaction ;;true if there is an interaction with another robot, false otherwise
  pLx ;; relative position respect to the leader (0 if on the same x axes, 1 if at his right, -1 if at his left)
  pLy ;; relative position respect to the leader (0 if on the same y axes, 1 if at his top, -1 if at his bottom)
  dL ;; distance from the leader
  lvl ;; Level of interaction, lvl=0 is the leader
  canChange ;;flag used to allow the level change
]

to setup-global-variables
  set GIR.radius 8 + 3.5 + 3.5    ;;setting it to 8 cm, plus two times half the dimension of a robot (3.5+3.5)
  set R 6 
  set dT 0.1 
  set previous_deployment Deployment 
  set Angle 0
  set G_force 0.3
  set LOPF_force 0.3
  set Homing_force 0.3  
  set Max_level 3
  set Noise_level 1
end

to setup-scouts
  create-scouts Number_of_robots
;;  ask scouts [ setxy random-xcor / 2 + 50 random-ycor / 2 + 50 set color grey]
  ask scouts [ setxy random-xcor random-ycor set color grey]
end

to setup-turtles
ask turtles [
  set size 9
  set IR.radius GIR.radius + random-normal 1 0.5 ;;the detection radius of each robot is changed adding a random term
  set epsilon G_Force
  set r R
  set pLx 0
  set pLy 0
  set dL 0
  set mass 0.05
  set Vmax 12.5 ;;maximum velocity [cm/s]
  set Fmax Vmax * mass / dT ;;� pari a 7 N
  set vel 0 ;;assuming robots start the simulation from a resting position
  set interaction FALSE
  set force array:from-list n-values 1 [0]
  correct_angle random 180
  set lvl 9
  set interLevel -1
  set canChange 1 ;;allowed to change
]
ask turtle 0[
  set xcor 100
  set ycor 100
  set vel 0
  if Deployment != "Hexagonal Lattice" and Deployment != "Square Lattice" and Deployment != "Line" [set color 40
  set lvl 0
  set ycor 125] ;;creating the Leader for Tree and Star deployments
]
end

to setup-patches
  ask patches  [set pcolor 33.1]
end


;;Beginning the simulation
to setup
  clear-all
  set-default-shape scouts "scout"
  setup-global-variables
  setup-scouts  ;;scouts setup
  setup-turtles  ;;setup for all the types of robots (other robots than the scouts could be added in the future)
  setup-patches
  reset-ticks
end

to go
  if not halo [ask patches [if pcolor = 56 [set pcolor 33.1]]]
  halo-movements
  force-effect
  if Deployment = "Tree" [barycenter-computation]
  update-sliders-parameters
  tick
end


;;Routine to compute the effect of the forces during the interactions between the robots (the effects of the LOPFs is computed in halo-movements)
to force-effect 
  
  ;;Variables needed to describe the interactions with the Leader
  let L 0 
  let Lx 0
  let Ly 0
  let bx 0
  let by 0
  
  ask patches  [if pcolor = 66 [set bx pxcor;; barycenter coordinates
                                set by pycor]] 
  ask turtles [if color = 40 [set L who
                              set Lx xcor
                              set Ly ycor]]
  ask turtles[  
    set canChange 1
    ifelse any? other turtles in-radius IR.radius and color != 40 and lvl != 0 [;; the Leader (lvl=0) is not affected by the other robots
      set interaction TRUE
      set pLx leader_x-position xcor Lx
      set pLy leader_y-position ycor Ly
      set dL vecotr-abs (xcor - Lx) (ycor - Ly)
    
      ;;needed to deal with the issue that Netlogo memorize the date from in-radius randomly, changing xcor and ycor
      let temp [who] of other turtles in-radius IR.radius
      set distanceX temp
      set distanceY temp
      set posX temp
      set posY temp
      let levels temp
      set interArray temp
      set temp array:from-list temp
      set distanceX array:from-list distanceX
      set distanceY array:from-list distanceY
      set posX array:from-list posX
      set posY array:from-list posY
      set levels array:from-list levels
      set interArray array:from-list interArray
      foreach n-values array:length temp [?] [
        array:set distanceX ? [xcor] of other turtles with [who = array:item temp ?]
        array:set distanceY ? [ycor] of other turtles with [who = array:item temp ?]
        array:set levels ? [lvl] of other turtles with [who = array:item temp ?]
      ]
      set force array:from-list n-values array:length distanceX [0]

      ;; Takes from the array of the distances the ?-th value and insert it at the ?-th position in the array force the resultant force
      foreach n-values array:length distanceX [?] [                      
           
           let templvl (item 0 array:item levels ?) ;;storing the minimum level of the neighbor robot, if the level is not 9
           if templvl != 9 [if interLevel < templvl [set interLevel templvl]]
           
           let check ((Lx = item 0 array:item distanceX ?) and (Ly = item 0 array:item distanceY ?)) ;; true when the interaction is with the Leader

           array:set posX ? (item 0 array:item distanceX ?) - bx + random-normal 0 Noise_level;;the error from the odometer sensor and the positioning system is simulated adding noise
           array:set posY ? (item 0 array:item distanceY ?) - by + random-normal 0 Noise_level 
            
           array:set distanceX ? (item 0 array:item distanceX ?) - xcor + random-normal 0 Noise_level ;;adding noise to simulate the error in the IR sensors measurements
           array:set distanceY ? (item 0 array:item distanceY ?) - ycor + random-normal 0 Noise_level 

           ;;Newtonian Law of Universal Gravitation 
           array:set force ? N-force (vecotr-abs array:item distanceX ? array:item distanceY ?) epsilon
           
           ifelse (Deployment = "Star") or (Deployment = "Tree") [             

               ;;In the Star deployment, robots belonging to level n interact with robots of level n and n-1
               if (Deployment = "Star") [ ifelse (lvl - item 0 array:item levels ? = 1) or (lvl - item 0 array:item levels ? = 0) [array:set interArray ? 1]
                                                                                                                                  [array:set interArray ? 0]
                                          if lvl = 9 [ifelse (lvl - item 0 array:item levels ? = 0) [array:set force ? -1 array:set interArray ? 1 set canChange 0]                                                                       
                                                                                                    [array:set force ? 0 array:set interArray ? 1 set canChange 1]]                                                                                        
                                                                                                                                  
                                                                                                                                  
                                        ]
               ;;In the Tree deployment, robots belonging to level n interact with robots of level n-1 and are 
               if (Deployment = "Tree") [ ifelse (lvl - item 0 array:item levels ? >= 0) [array:set interArray ? 1]
                                                                                         [array:set interArray ? 0]
                                          if (lvl - item 0 array:item levels ? = 0) [array:set force ? -1 array:set interArray ? 1]
                                          if lvl = 9 [ifelse (lvl - item 0 array:item levels ? = 0) or (Max_level = item 0 array:item levels ?)  [array:set force ? -1 array:set interArray ? 1 set canChange 0]                                                                       
                                                                                                                                               [array:set force ? 0 array:set interArray ? 1 set canChange 1]]

                                        ]

               ;;Robots belonging to level 9 get repulsed from other robots belonging to lvl 9,
               ;;and suffer only of the LOPF when interacting with robots of other levels (no Newtonian interactions)
               ;;The variable canChange is used so that a robot can change level only when it interacts with robots of another level

           ][array:set interArray ? 1]
                                                                                                                            
      ]
    ]
    [set interaction FALSE
        foreach n-values array:length force [?] [  
        array:set force ? 0
        ]
      set distanceX []
      set distanceY []
    ]
]    
end

  

to halo-movements
 let L 0 ;;needed for the interactions with the Leader
 let Lx 0
 let Ly 0
 let bx 0 ;;barycenter coordinates
 let by 0
 ask patches  [if pcolor = 66 [set bx pxcor
                               set by pycor]] 
 ask turtles [if color = 40 [set L who
                             set Lx xcor
                             set Ly ycor]]
  ask turtles[;;the halo is destroyed
              if halo [ask patches in-radius IR.radius [set pcolor 33.1]]                                        
              
              ;;if there is an interaction
              ifelse interaction [

              ;;changing the levels
              if canChange = 1 [if color != 40 [set lvl change-level atan (Ly - ycor) (Lx - xcor) pLx pLy dL]] ;; the Leader never changes
              
                   foreach n-values array:length force [?] [
                      let Tv array:item force ? ;;utility variable
                      ;;implementing the effect of the forces
                          if abs(Tv) > Fmax [set Tv Fmax * Tv / abs(Tv)] ;;setting to Fmax, keeping the sign
                          set vel friction vel
                          let vx vel * cos(alpha)
                          let vy vel * sin(alpha)
                          ;;initialization
                          let dvx 0
                          let dvy 0
                          let fvx 0
                          let fvy 0                    
           
                          if array:item interArray ? = 1 [
                            if Deployment != "Tree"  and Deployment != "Star" [ 
                              set dvx Tv * dT * cos(atan array:item distanceY ? array:item distanceX ?) / mass
                              set dvy Tv * dT * sin(atan array:item distanceY ? array:item distanceX ?) / mass
                              set fvx latticex dvx atan array:item distanceY ? array:item distanceX ? ;; Adding the effect of the LOPF
                              set fvy latticey dvy atan array:item distanceY ? array:item distanceX ?] ;; Adding the effect of the LOPF

                            if array:item interArray ? = 1 and Deployment != "Hexagonal Lattice"  and Deployment != "Square Lattice" and Deployment != "Line"[
                              set dvx Tv * dT * cos(atan array:item distanceY ? array:item distanceX ?) / mass
                              set dvy Tv * dT * sin(atan array:item distanceY ? array:item distanceX ?) / mass
                              set fvx Deploymentx atan array:item distanceY ? array:item distanceX ? pLx pLy dL array:item posX ? array:item posY ? ;; Adding the effect of the LOPF
                              set fvy Deploymenty atan array:item distanceY ? array:item distanceX ? pLx pLy dL array:item posX ? array:item posY ?] ;; Adding the effect of the LOPF
                             ] 
                                    
                          let fhx 0
                          let fhy 0
                          
                          if Homing = 1 and  Deployment != "Star" and  Deployment != "Tree" and lvl = 9[
                             set fhx (100 - xcor + 0.0001) * Homing_Force / 100 ;;0.0001 is used to avoid dividing for 0 when computing the force on the leader
                             set fhy (100 - ycor + 0.0001) * Homing_Force / 100]                           
                          
                          if Homing = 1 and  Deployment = "Star" and lvl = 9[
                             set fhx (Lx - xcor + 0.0001) * Homing_Force / 100 ;;0.0001 is used to avoid dividing for 0 when computing the force on the leader
                             set fhy (Lx - ycor + 0.0001) * Homing_Force / 100] 

                          ;;in the case of Tree or Star pattern the robots are attracted to the barycenter of the formation (the leader is the only one not affected)
                          if Homing = 1 and  Deployment = "Tree" and lvl = 9 [ 
                             set fhx (bx - xcor + 0.0001) * Homing_Force / 100
                             set fhy (by - ycor + 0.0001) * Homing_Force / 100]                 
                          set vx vx + dvx + fvx + fhx
                          set vy vy + dvy + fvy + fhy
                          if abs(vx) > Vmax [set vx Vmax * vx / abs(vx)]
                          if abs(vy) > Vmax [set vy Vmax * vy / abs(vy)] 
                          if vx + vy != 0 [
                            correct_angle atan vy vx]             
                                                             
                          set vel vecotr-abs vx vy ;;new absolute value of the velocity
                          let spostamentox vx * dT
                          let spostamentoy vy * dT
                          setxy (xcor + spostamentox) (ycor + spostamentoy)
                     ]
              ] ;;If no interaction: (there is no friction, so the robots can keep moving)
              [let fhx 0
               let fhy 0               
               if who != L [set lvl 9 set color grey]               
               set interArray 0
               if Homing = 1 and  Deployment != "Star" and  Deployment != "Tree" and lvl = 9[
                             set fhx (100 - xcor + 0.0001) * Homing_Force / 100 ;;0.0001 is used to avoid dividing for 0 when computing the force on the leader
                             set fhy (100 - ycor + 0.0001) * Homing_Force / 100]            
               if Homing = 1 and Deployment = "Star" and lvl = 9[
                             set fhx (Lx - xcor + 0.0001) * Homing_Force / 100
                             set fhy (Ly - ycor + 0.0001) * Homing_Force / 100] 
               if Homing = 1 and  Deployment = "Tree" and lvl = 9[ 
                             set fhx (bx - xcor + 0.0001) * Homing_Force / 100 ;;the robots are attracted toward the barycenter
                             set fhy (by - ycor + 0.0001) * Homing_Force / 100
                             set color grey  ;; the robots that are not interacting join level 9
                            ; set lvl 9
                             ]                                                                                                                     
               ;;if random 100 >= 99 [ ;;si ha una probabilit� del 1% di spostarsi in una direzione a caso ripartendo con una certa velocit�           
                 ;;correct_angle random 360]
               let vx vel * cos(alpha) + fhx 
               let vy vel * sin(alpha) + fhy
               if vx + vy != 0 [
                correct_angle atan vy vx]
               set vel vecotr-abs vx vy ;;new absolute value of the velocity       
               let spostamentox vx * dT
               let spostamentoy vy * dT
               setxy (xcor + spostamentox) (ycor + spostamentoy)
               
              ]
                          
              ;;creating the halo
              if halo [
                 let tx xcor
                 let ty ycor
                 let I IR.radius
                 ask patches in-radius IR.radius [
                   if distancexy tx ty > (I - 0.3) and distancexy tx ty <= I [set pcolor 56]            
                 ]] 

  ]
end
  
  
to update-sliders-parameters
let bx 0
let by 0
ask patches  [if pcolor = 66 [set bx pxcor;; barycenter coordinates
                              set by pycor]] 
if Homing = 1 and Deployment != "Tree" and Deployment != "Star" [
    ask patches  [if pxcor = 100 and pycor = 100 [set plabel "home"]]]
if Homing = 0 [
    ask patches  [if pxcor = 100 and pycor = 100 [set plabel ""]]]          

if previous_deployment != Deployment[  
  set previous_deployment Deployment
  set Angle 0

  if Deployment = "Tree" or Deployment = "Star" [ask turtles [set color grey set lvl 9] 
    ask turtle 0 [ set color 40 set lvl 0 set vel 0 set interaction False]
    ask patches  [if pxcor = 100 and pycor = 100 [set plabel ""]]
    ]
    
  if Deployment = "Hexagonal Lattice" or Deployment = "Square Lattice" or  Deployment = "Line"  [ask turtles [ set color grey set lvl 9  ]    
    ]
  
  
  ]
                                                                                            
ask turtles [
  set epsilon G_Force
  ]
end

to correct_angle[ang] ;; correctin the angle for used the reference system
  set alpha ang
  set heading 90 - ang
end
  
to-report friction[fv]  ;;reducing the speed because of the friction
ifelse (fv != 0 and abs(fv) >= 1.5)[
   report fv - 1.5
]
[report 0]
end

;;Local Oriented Potential Field effect
to-report latticex[ax a]
if Deployment = "Hexagonal Lattice" [report 0]

if Deployment = "Square Lattice" [if abs(((a mod 45 - 22.5) / 22.5)) < 0.9 [ ;;this condition is needed to avoid robots getting stuck in the middle of two different ares of the LOPF
          if (a >= 45 and a < 90) or (a >= 270 and a < 315) [report -6 * LOPF_Force] ;;per LOPF_Force=0.64 si passa dalla formazione esagonale a quella quadrata con facilit�, altrimenti LOPF_Force=0.51 � sufficiente (ed ha pure la propriet� di mantenere il sistema stabile)
          if (a >= 90 and a < 135) or (a >= 225 and a < 270) [report 6 * LOPF_Force]] ;;un'idea potrebbe essere quella di fare partire LOPF_Force da 0.64 e farlo diminuire fino a 0.51 col frame_delay. Idem per G, che parte da 20 o 24, e diminuisce fino a 3.
          report 0] ;;caso quadrato
          
if Deployment = "Line" [if abs(((a mod 90 - 45) / 45)) < 0.9 [ ;;this condition is needed to avoid robots getting stuck in the middle of two different ares of the LOPF
          if (a >= 0 and a < 90) or (a >= 270 and a < 360) [report -6 * LOPF_Force]
          if (a >= 90 and a < 270) [report 6 * LOPF_Force]]
          report 0]
end
          
;;Local Oriented Potential Field effect
to-report Deploymentx[a p_x p_y d_ p2_x p2_y]
set a (a + 180) mod 360 ;; adapting the angle to the used reference system

if Deployment = "Tree" [
          if p_x = 0[                   
              if  (a >= 90 and a < 180) [report -2 * LOPF_Force]    
              if  (a >= 0 and a < 90) [report 2 * LOPF_Force]]
          if p_x = 1[                   
              if  (a >= 90 and a < 180) [report 2 * LOPF_Force]    
              if  (a >= 0 and a < 90) [report 2 * LOPF_Force]]
          if p_x = -1[                   
              if  (a >= 90 and a < 180) [report -2 * LOPF_Force]    
              if  (a >= 0 and a < 90) [report -2 * LOPF_Force]]
          
          if abs((((a + 45) mod 90 - 45) / 45)) < 0.95  [
              if (a >= 225 + Angle and a < 270) or (a >= 315 - Angle and a < 360) [report -2 * LOPF_Force]
              if (a >= 180 and a < 225 + Angle) or (a >= 270 and a < 315 - Angle) [report 2 * LOPF_Force]                              
      ]
          report 0]       

if Deployment = "Star" [

          if (dL / (R + 7)) < 1.3 [
          if abs((((a + 30) mod 120 - 60) / 60)) < 0.95  [
              if (a >= 0 and a < 90) or (a >= 210 and a < 270) or (a >= 330 and a < 360) [report -2 * LOPF_Force]
              if (a >= 90 and a < 210) or (a >= 270 and a < 330) [report 2 * LOPF_Force]]]
          if (dL / (R + 7)) >= 1.3  [
             if (p_x = -1 or p_x = 0) and (p_y = -1 or p_y = 0) [ 
                  if (a >= 30 and a < 210 + Angle) [report 2 * cos(60 - Angle) * LOPF_Force]
                  if (a >= 210 + Angle and a < 360) or (a >= 0 and a < 30) [report -2 * cos(60 - Angle) * LOPF_Force]]
              if p_x = 1 and (p_y = -1 or p_y = 0) [ 
                  if (a >= 150 and a < 330 - Angle) [report 2 * cos(60 - Angle) * LOPF_Force]
                  if (a >= 0 and a < 150) or (a >= 330 - Angle and a < 360) [report -2 * cos(60 - Angle) * LOPF_Force]]     
              if p_y = 1 [ 
                  if (a >= 0 and a < 90) or (a >= 270 and a < 360) [report -2 * LOPF_Force]          
                  if (a >= 90 and a < 270) [report 2 * LOPF_Force]]]
          report 0]          
end

;;Local Oriented Potential Field effect
to-report latticey[ay a]
if Deployment = "Hexagonal Lattice" or Deployment = "Line" [report 0] ;;caso esagonale

if Deployment = "Square Lattice" [if abs(((a mod 45 - 22.5) / 22.5)) < 0.9 [
          if (a >= 0 and a < 45) or (a >= 135 and a < 180) [report -6 * LOPF_Force]
          if (a >= 180 and a < 225) or (a >= 315 and a < 360) [report 6 * LOPF_Force]]
          report 0] ;;caso quadrato
end

;;Local Oriented Potential Field effect
to-report Deploymenty[a p_x p_y d_ p2_x p2_y]
set a (a + 180) mod 360 ;; si ristabilisce la giusta prospettiva per il calcolo delle forze
if Deployment = "Tree" [
          if  (a >= 0 and a < 180) [report -2 * LOPF_Force]    
          if abs((((a + 45) mod 90 - 45) / 45)) < 0.95  [                        
              if (a >= 180 and a < 225 + Angle) or (a >= 315 - Angle and a < 360) [report -2 * LOPF_Force]
              if (a >= 225 + Angle and a < 270) or (a >= 270 and a < 315 - Angle) [report 2 * LOPF_Force]                                                                                     
             ]
          
          report 0]
    

if Deployment = "Star" [
          if (dL / (R + 7)) < 1.3[
          if p_x = 0 and p_y = -1 [report -1]
              if abs((((a + 30) mod 120 - 60) / 60)) < 0.95 [
                  if (a >= 180 and a < 210) or (a >= 330 and a < 360) [report -2 * LOPF_Force]
                  if (a >= 210 and a < 330) or (a >= 0 and a < 180) [report 2 * LOPF_Force]]]
          if (dL / (R + 7)) >= 1.3 [
              if p_x = -1 and (p_y = -1 or p_y = 0) [ 
                  if (a >= 30 and a < 210 + Angle) [report -2 * sin(60 - Angle) * LOPF_Force]
                  if (a >= 210 + Angle and a < 360) or (a >= 0 and a < 30) [report 2 * sin(60 - Angle) * LOPF_Force]]
              if p_x = 1 and (p_y = -1 or p_y = 0) [ 
                  if (a >= 150 and a < 330 - Angle) [report 2 * sin(60 - Angle) * LOPF_Force]
                  if (a >= 0 and a < 150) or (a >= 330 - Angle and a < 360) [report -2 * sin(60 - Angle) * LOPF_Force]]
              if p_y = 1 [ 
                  if (a >= 0 and a < 90) [report 0 * LOPF_Force]          
                  if (a >= 90 and a < 180) [report 0 * LOPF_Force]]]       
report 0]          
end


;; rutine needed to understand in which are of the LOPF coordinate system the leader is placed 
to-report leader_x-position[ax lx]
ifelse abs(ax - lx) > 5 [report (ax - lx) / abs (ax - lx)] ;; -1 is left, +1 is right
       [report 0]
end

;; rutine needed to understand in which are of the LOPF coordinate system the leader is placed 
to-report leader_y-position[ay ly]
ifelse abs(ay - ly) > 5 [report (ay - ly) / abs (ay - ly)] ;; -1 is below, +1 is above
       [report 0]
end


;;rutine to add a new robot
to add_robot
  let a FALSE
  while [mouse-down?]
    [ if not a[
      create-scouts 1
      [setxy mouse-xcor mouse-ycor set color grey
      set size 9
      set IR.radius GIR.radius + random-normal 1 0.5
      set epsilon G_Force
      set r R
      set pLx 0
      set pLy 0
      set dL 0
      set lvl 9
      set mass 0.1 
      set Vmax 12.5 ;;[cm/s]
      set Fmax Vmax * mass / dT 
      set vel 0 ;;assuming new robots are standing still
      set interaction FALSE
      set force array:from-list n-values 1 [0]
      correct_angle random 180]
      display
      set a TRUE
     ]  ]
end


;;test: show abs(((((ac + 180) mod 360 + 30) mod 120 - 60) / 60))
to-report change-level [ac pc_x pc_y dc_] ;;needed to assign the level to the robots in the Star and Tree deployments


set ac (ac + 180) mod 360 
let newlvl 9
if Deployment = "Star" [
        if (dc_ / (R + 7)) < 1.3 and abs((((ac + 30) mod 120 - 60) / 60)) >= 0.9 [set newlvl 1]
        if Angle = 0 [if (dc_ / (R + 7)) >= 1.3 and abs((((ac + 30) mod 120 - 60) / 60)) >= 0.9 [set newlvl 2]] ;;the thresholds need to be changed if a higher number of robots is used
        if Angle != 0 [if (dc_ / (R + 7)) >= 1.3 [set newlvl 2]] ;;the thresholds need to be changed if a higher number of robots is used
  ]
if Deployment = "Tree" [
        let dLx dc_ * cos(ac)
        let dLy dc_ * sin(ac) 
        if (dc_ / (R + 7)) < 1.5 [
           if Angle != 0 [set newlvl 1]
           if Angle = 0 [if (ac >= 180 and ac < 360) and abs((((ac + Angle + 45) mod 90 - 45) / 45)) >= 0.85 [set newlvl 1]]
        ]
        if Angle > 11 [set Angle 11]
        let threshold_y (R + 6.5) * cos(45 - Angle) 
        let threshold_x (R + 10) * sin(45 - Angle)
        let range round (abs(dLy) / threshold_y) ;;utility variable to assign the level
        if (dc_ / (R + 7)) >= 1.3 and range != 1 and abs(dLx) <= abs(threshold_x * range) [ 
           if (ac >= 180 and ac < 360) [set newlvl range set color (10 * range + 15)]
           if range > Max_level [set newlvl 9]
        ]                     
  ]
if newlvl = 1 [set color 15]
if newlvl = 2 [set color 25]
if newlvl = 3 [set color 45] 
if newlvl = 9 [set color grey]

report newlvl

end

to barycenter-computation
let n 3
let a 0
let b 0
ask turtles [if color = 40 [ set a xcor
                             set b ycor]]
;;computing the apotema
let L (R + 7) / (2 * cos(180 * (n - 1) / (2 * (n + 1))) )                    
                                
ask patches  [if pxcor = ceiling a and pycor = ceiling (b - L) [set pcolor 66]];; [set pcolor 33.1]]                                                                                                
                              
end


;;Newtonian Law of Universal Gravitation 
to-report N-force [rN Nepsilon]
 ifelse rN >= R + 7 [report 15 * Aggregation * Nepsilon / (rN ^ 0.75)] 
                 [report -10 * Dispersion * Nepsilon / (rN ^ 0.75)]
end


to-report vecotr-abs[xmod1 ymod1]
  report sqrt (xmod1 ^ 2 + ymod1 ^ 2)
end

to-report vecotr-abs-modificato[xmod ymod] ;;it is needed to take into account that the true position of the robots needs to be diminished by 3.5 + 3.5 cm (two times half the size of a robot)
  ifelse xmod > 0 [set xmod xmod - 7]      
  [set xmod xmod + 7]
  ifelse ymod > 0 [set ymod ymod - 7]
  [set ymod ymod + 7]
  report sqrt (xmod ^ 2 + ymod ^ 2)
end
@#$#@#$#@
GRAPHICS-WINDOW
559
12
1152
626
-1
-1
2.9005
1
10
1
1
1
0
1
1
1
0
200
0
200
1
1
1
ticks
30.0

BUTTON
23
72
149
105
Setup Agents
Setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
24
116
148
149
Move Agents
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
172
72
344
105
Number_of_robots
Number_of_robots
1
10
10
1
1
NIL
HORIZONTAL

BUTTON
23
163
148
196
Move Once
Go
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SLIDER
376
73
548
106
G_Force
G_Force
0
1
0.3
0.1
1
NIL
HORIZONTAL

SLIDER
376
120
548
153
LOPF_Force
LOPF_Force
0.1
1
0.3
0.1
1
NIL
HORIZONTAL

BUTTON
24
206
149
239
Add Robot
Add_robot
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

TEXTBOX
198
278
348
296
Coordination flags:\n
11
0.0
1

SLIDER
171
308
343
341
Aggregation
Aggregation
0
1
1
1
1
NIL
HORIZONTAL

SLIDER
171
354
343
387
Dispersion
Dispersion
0
1
1
1
1
NIL
HORIZONTAL

SLIDER
375
168
547
201
R
R
4
GIR.radius + 3 - 7 - 2
6
1
1
NIL
HORIZONTAL

SLIDER
171
397
343
430
Homing
Homing
0
1
1
1
1
NIL
HORIZONTAL

CHOOSER
174
122
344
167
Deployment
Deployment
"Hexagonal Lattice" "Square Lattice" "Line" "Star" "Tree"
3

TEXTBOX
412
40
562
58
Force parameters:
11
0.0
1

TEXTBOX
398
279
548
297
Deployment properties:
11
0.0
1

TEXTBOX
213
42
316
60
Basic inputs:
11
0.0
1

SLIDER
374
310
546
343
Angle
Angle
-4
30
0
1
1
NIL
HORIZONTAL

SLIDER
375
397
547
430
Noise_level
Noise_level
0
10
1
0.1
1
NIL
HORIZONTAL

SWITCH
375
442
547
475
Halo
Halo
1
1
-1000

SLIDER
375
216
547
249
Homing_Force
Homing_Force
0
1
0.3
0.1
1
NIL
HORIZONTAL

SLIDER
375
352
547
385
Max_level
Max_level
1
6
3
1
1
NIL
HORIZONTAL

@#$#@#$#@
## WHAT IS IT?

This program shows a group of robots creating five different deployments: Hexagonal, Square, Line, Star and Tree.

## HOW IT WORKS

A group of robots, depicted as square of 7cm, moves through an arena of 2x2 m^2. Different deployment can be tested, so as different cohordination properties.  
The robot interact using two type of forces: the Artificial Physics and the force from Local Oriented Potential Fields (a potential field centered on each robot of the simulation and affecting its neighbors).
   
## HOW TO USE IT

SETUP sets up the model according to the values indicated by all the sliders and the switch. GO is a forever button that executes the model continually. Press the Add robot button and click on the arena to add a robot in the selected point.

Basic Inputs:  
- Number_Of_Robots affects the initial set of robots.   
- Deployment decides which deployment the robots will assume.

Force Parameters:  
- G_Force represents the Gravitational constant of the Newton's Law of Universal Gravitation (used by the Artificial Physics paradigm)  
- LOPF_Force represents the force arising from the Local Oriented Potential Fields paradigm  
- R represents the radius of the Newton's Law of Universal Gravitation (used by the Artificial Physics paradigm)  
- Homing_Force affects the force attracting the robots to their home (either the center of the arena, the leader or the barycenter)

Coordination Flags:  
- Aggregation lets the aggregation behavior between the robots be not active (0) or active (1)  
- Dispersion lets the dispersion behavior between the robots be not active (0) or active (1)  
- Homing lets the dispersion behavior between the robots be not active (0) or active (1)

Deployment Properties:  
- Angle can be used to change the angle between the robots when they are deployed in the Star and the Tree deployment (while changing it, give time to the robots to re-assemble!)  
- Max_level corresponds to the maximum number of raws composing the Tree deployment (try to change its value to avoid robots getting stucked in local optima)  
- Noise_level affects the level of noise disturbing the movements of the robots  
- Halo shows the detection radius of the robots (represents the range of the proximity sensors)

This model has been constructed so that all changes in the sliders and switches will take effect in the model during execution. So, while the GO button is still down, you can change the values of the sliders and the switch, and you can see these changes immediately in the view.
   
## THINGS TO KNOW

The Star and the Tree deployments make use of a routine assigning a level to each robot depending on their position (angle and distance) respect to the Leader. The Leader is the robot in black, and by definition is not affected by any force. Each level corresponds to a different color.

## THINGS TO TRY

Study how different initial position of the robots affect the deployment. For instance, try to start with a certain number of robots and then add some others through the Add robots button while the simulation is running. Then, compare the perfomance when you use as initial position another deployment. For instance, wait for the robots to deploy as an hexagon and then call the star deployment. Which is the best way to start the deployment?

## HOW TO CITE

If you mention this model in an academic publication, we ask that you include these citations for the model itself and for the NetLogo software:  
- Spears, W. M. and Spears, D. F. (eds.) Physics-based Swarm Intelligence: From Theory to Practice, Springer-Verlag, (2011).  
- Wilensky, U. (1999). NetLogo. http://ccl.northwestern.edu/netlogo/. Center for Connected Learning and Computer-Based Modeling, Northwestern University, Evanston, IL.

## COPYRIGHT NOTICE

Copyright 2011 Andrea Bravi. All rights reserved.

Permission to use, modify or redistribute this model is hereby granted, provided that both of the following requirements are followed:  
a) this copyright notice is included.  
b) this model will not be redistributed for profit without permission from Andrea Bravi. Contact Andrea Bravi for appropriate licenses for redistribution for profit.

http://aix2.uottawa.ca/~abrav103/
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

jasmine
true
0
Rectangle -7500403 true true 30 30 270 270
Circle -2674135 true false 135 30 30
Circle -13791810 true false 24 24 42
Circle -13791810 true false 24 129 42
Circle -13791810 true false 135 240 30
Circle -13791810 true false 234 234 42
Circle -13791810 true false 24 234 42
Circle -13791810 true false 129 234 42
Circle -13791810 true false 234 129 42
Circle -13791810 true false 234 24 42
Circle -1 false false 135 30 30
Circle -16777216 false false 24 24 42
Circle -16777216 false false 234 24 42
Circle -16777216 false false 24 129 42
Circle -16777216 false false 24 234 42
Circle -16777216 false false 129 234 42
Circle -16777216 false false 234 234 42
Circle -16777216 false false 234 129 42

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

rscout
false
0
Rectangle -13791810 true false 30 30 270 270
Rectangle -16777216 true false 135 30 165 60

rtank
false
0
Rectangle -2674135 true false 30 30 270 270
Rectangle -16777216 true false 135 30 165 60

scout
true
0
Rectangle -7500403 true true 30 30 270 270
Circle -2674135 true false 135 30 30
Circle -13791810 true false 24 24 42
Circle -13791810 true false 24 129 42
Circle -13791810 true false 135 240 30
Circle -13791810 true false 234 234 42
Circle -13791810 true false 24 234 42
Circle -13791810 true false 129 234 42
Circle -13791810 true false 234 129 42
Circle -13791810 true false 234 24 42
Circle -1 false false 135 30 30
Circle -16777216 false false 24 24 42
Circle -16777216 false false 234 24 42
Circle -16777216 false false 24 129 42
Circle -16777216 false false 24 234 42
Circle -16777216 false false 129 234 42
Circle -16777216 false false 234 234 42
Circle -16777216 false false 234 129 42

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

tank
true
0
Rectangle -7500403 true true 30 30 270 270
Circle -2674135 true false 135 30 30
Circle -955883 true false 24 24 42
Circle -955883 true false 24 129 42
Circle -13791810 true false 135 240 30
Circle -955883 true false 234 234 42
Circle -955883 true false 24 234 42
Circle -955883 true false 129 234 42
Circle -955883 true false 234 129 42
Circle -955883 true false 234 24 42
Circle -1 false false 135 30 30
Circle -16777216 false false 24 24 42
Circle -16777216 false false 234 24 42
Circle -16777216 false false 24 129 42
Circle -16777216 false false 24 234 42
Circle -16777216 false false 129 234 42
Circle -16777216 false false 234 234 42
Circle -16777216 false false 234 129 42

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 5.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
