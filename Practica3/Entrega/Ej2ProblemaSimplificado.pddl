﻿(define (problem Problema2Simplificado)

(:domain BELKAN2)

(:OBJECTS
zona1_1 zona1_2 zona1_3 zona1_4 zona1_5
zona2_1 ;zona2_2
zona2_3 ;zona2_4
zona2_5
zona3_1 zona3_2 zona3_3 zona3_4 zona3_5
zona4_1
;zona4_2
zona4_3
;zona4_4
zona4_5
zona5_1 zona5_2 zona5_3 zona5_4 zona5_5
 - zona
    ROBOT - robot OSCAR MANZANA ROSA ALGORITMO ORO - objeto PRINCESA PRINCIPE BRUJA
	PROFESOR LEONARDO - personaje sur norte este oeste - orientacion)

(:INIT

(conectada zona1_1 zona3_1 sur)
(conectada zona1_1 zona1_3 este)

;(conectada zona1_2 zona2_2 sur)
;(conectada zona1_2 zona1_3 este)
;(conectada zona1_2 zona1_1 oeste)
(conectada zona1_3 zona3_3 sur)
(conectada zona1_3 zona1_5 este)
(conectada zona1_3 zona1_1 oeste)

;(conectada zona1_4 zona2_4 sur)
;(conectada zona1_4 zona1_5 este)
;(conectada zona1_4 zona1_3 oeste)
(conectada zona1_5 zona3_5 sur)
(conectada zona1_5 zona1_3 oeste)
;(conectada zona2_1 zona1_1 norte)
;(conectada zona2_1 zona3_1 sur)
;(conectada zona2_1 zona2_2 este)
;(conectada zona2_2 zona1_2 norte)
;(conectada zona2_2 zona3_2 sur)
;(conectada zona2_2 zona2_3 este)
;(conectada zona2_2 zona2_1 oeste)
;(conectada zona2_3 zona1_3 norte)
;(conectada zona2_3 zona3_3 sur)
;(conectada zona2_3 zona2_4 este)
;(conectada zona2_3 zona2_2 oeste)
;(conectada zona2_4 zona1_4 norte)
;(conectada zona2_4 zona3_4 sur)
;(conectada zona2_4 zona2_5 este)
;(conectada zona2_4 zona2_3 oeste)
;(conectada zona2_5 zona1_5 norte)
;(conectada zona2_5 zona3_5 sur)
;(conectada zona2_5 zona2_4 este)
(conectada zona3_1 zona1_1 norte)
(conectada zona3_1 zona5_1 sur)
;(conectada zona3_1 zona3_2 este)
;(conectada zona3_2 zona2_2 norte)
;(conectada zona3_2 zona4_2 sur)
;(conectada zona3_2 zona3_3 este)
;(conectada zona3_2 zona3_1 oeste)
(conectada zona3_3 zona1_3 norte)
(conectada zona3_3 zona4_3 sur)
;(conectada zona3_3 zona3_4 este)
;(conectada zona3_3 zona3_2 oeste)
;(conectada zona3_4 zona2_4 norte)
;(conectada zona3_4 zona4_4 sur)
;(conectada zona3_4 zona3_5 este)
;(conectada zona3_4 zona3_3 oeste)
(conectada zona3_5 zona1_5 norte)
(conectada zona3_5 zona5_5 sur)
;(conectada zona3_5 zona3_4 este)
;(conectada zona4_1 zona3_1 norte)
;(conectada zona4_1 zona5_1 sur)
;(conectada zona4_1 zona4_2 este)
;(conectada zona4_2 zona3_2 norte)
;(conectada zona4_2 zona5_2 sur)
;(conectada zona4_2 zona4_3 este)
;(conectada zona4_2 zona4_1 oeste)
(conectada zona4_3 zona3_3 norte)
(conectada zona4_3 zona5_3 sur)
;(conectada zona4_3 zona4_4 este)
;(conectada zona4_3 zona4_2 oeste)
;(conectada zona4_4 zona3_4 norte)
;(conectada zona4_4 zona5_4 sur)
;(conectada zona4_4 zona4_5 este)
;(conectada zona4_4 zona4_3 oeste)
;(conectada zona4_5 zona3_5 norte)
;(conectada zona4_5 zona5_5 sur)
;(conectada zona4_5 zona4_4 este)
(conectada zona5_1 zona3_1 norte)
(conectada zona5_1 zona5_3 este)
;(conectada zona5_2 zona4_2 norte)
;(conectada zona5_2 zona5_3 este)
;(conectada zona5_2 zona5_1 oeste)
(conectada zona5_3 zona4_3 norte)
(conectada zona5_3 zona5_5 este)
(conectada zona5_3 zona5_1 oeste)
;(conectada zona5_4 zona4_4 norte)
;(conectada zona5_4 zona5_5 este)
;(conectada zona5_4 zona5_3 oeste)
(conectada zona5_5 zona3_5 norte)
(conectada zona5_5 zona5_3 oeste)

;Costes



(= (coste zona1_1 zona3_1 ) 10)
(= (coste zona1_1 zona1_3 ) 15)

(= (coste zona1_3 zona3_3 ) 10)
(= (coste zona1_3 zona1_5 ) 15)
(= (coste zona1_3 zona1_1 ) 9)

(= (coste zona1_5 zona3_5 ) 10)
(= (coste zona1_5 zona1_3 ) 9)

(= (coste zona3_1 zona1_1 ) 13)
(= (coste zona3_1 zona5_1 ) 10)

(= (coste zona3_3 zona1_3 ) 13)
(= (coste zona3_3 zona4_3 ) 10)

(= (coste zona3_5 zona1_5 ) 13)
(= (coste zona3_5 zona5_5 ) 10)

(= (coste zona4_3 zona3_3 ) 13)
(= (coste zona4_3 zona5_3 ) 10)

(= (coste zona5_1 zona3_1 ) 13)
(= (coste zona5_1 zona5_3 ) 15)

(= (coste zona5_3 zona4_3 ) 13)
(= (coste zona5_3 zona5_5 ) 15)
(= (coste zona5_3 zona5_1 ) 9)

(= (coste zona5_5 zona3_5 ) 13)
(= (coste zona5_5 zona5_3 ) 9)



(izq sur este)
(izq este norte)
(izq norte oeste)
(izq oeste sur)


    (atP LEONARDO zona1_5)
    (atP PROFESOR zona3_5)
    (atP PRINCIPE zona5_5)
		(atP BRUJA zona1_1)
    (atP PRINCESA zona3_1)

		(atO OSCAR zona3_3)
		(atO MANZANA zona4_3)
    (atO ROSA zona5_3)
    (atO ALGORITMO zona5_1)
    (atO ORO zona1_3)


		(atR ROBOT zona5_1)
		(orientado ROBOT este)
		(MANOVACIA)
    (= (coste-total) 0)
)

(:goal
      (AND
          (tiene ORO PRINCIPE)
          (tiene OSCAR LEONARDO)
          (tiene MANZANA BRUJA)
          (tiene ALGORITMO PROFESOR)
          (tiene ROSA PRINCESA)
      )
)
(:metric minimize (coste-total))

)
