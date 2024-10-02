### TODO

-Generar interacción para encontrar personas previamente encontradas

-Revisar puertos de los BTs

-Mirar si se puede formatear la salida del speak_result.

-Hacer movimiento antes del speak en el subárbol de speak_result no es válido en algunos planes (Los 70b siempre hacen un move_to antes del speak_result, los 7b no)

-A veces no considera los muebles como waypoints


-Ver que hacer con esto (que busque persona o no?)
tell the gesture of the person at the coathanger to the person at the lamp

move_to ['coathanger']

describe_person ['arm_gesture']

move_to ['lamp']

speak_result []