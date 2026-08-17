# Sobre los tiempos de los videos y los logs

Juan Carlos:

Revisando el material del 30 de julio confirmé que la duración de los videos no
coincide con la de los logs de posición, y quería explicar a qué se debe para que
no quede como una inconsistencia de los datos.

El desfase es del programa de grabación, no de la medición. La base escribe un
fotograma de video por cada fotograma de cámara que procesa, pero declara la tasa
del archivo con un valor fijo calculado de antemano: 19 cuadros por segundo, que
sería la tasa si cada ciclo tardara exactamente el intervalo de procesamiento
configurado. En la práctica cada ciclo tarda más, porque además del intervalo hay
que sumar la detección de los marcadores ArUco —que varía según cuántos
marcadores estén visibles en ese instante—, el dibujo del mapa y el envío por
red. El resultado es que se graban menos cuadros por segundo de los que el
archivo declara, y el reproductor los pasa más rápido de lo que ocurrieron.

Lo medí sobre las quince corridas. La tasa real de grabación estuvo entre 10.3 y
15.2 cuadros por segundo contra los 19 declarados, así que los videos corren
entre un 25 % y un 84 % más rápido que la realidad, según la corrida. El caso más
extremo es una corrida que dura 104 segundos de video y 192 segundos de log. La
variación entre corridas tiene sentido con la explicación: las corridas donde la
cámara pierde marcadores con más frecuencia son las que más se alejan, porque son
las que más trabajo de detección exigen por cuadro.

Lo importante es que **el dato válido es el del log**. Las posiciones se
registran con la marca de tiempo del momento en que se capturó el fotograma, así
que la escala temporal de los logs es correcta; lo único equivocado es la
velocidad de reproducción del video. De hecho el reloj que aparece sobreimpreso
en cada cuadro sale de esa misma marca de tiempo, de modo que también es
confiable: si uno pausa el video y lee el reloj en pantalla, ese número sí
corresponde al log. Lo que no sirve es el contador del reproductor.

Para ubicar un instante determinado conviene entonces guiarse por el reloj
sobreimpreso, o por número de cuadro: la proporción entre cuadros y duración del
log es aproximadamente uniforme dentro de una misma corrida. Lo verifiqué al
reconstruir la geometría de los escenarios a partir de los videos, y la
calibración cerró con un error mediano de 3 a 6 milímetros, lo cual solo es
posible si esa proporción se sostiene. Eso sí, el factor cambia bastante de una
corrida a otra, así que hay que calcularlo por corrida y no fijarlo de una vez.

Lo vamos a corregir volviendo a empaquetar el archivo al terminar cada corrida
con la tasa realmente medida. No alcanza con cambiar la constante, porque la tasa
depende de la carga de procesamiento de cada escena y no se conoce hasta que la
corrida termina.

Aprovecho para señalar algo relacionado que también afecta cómo se leen los
tiempos, y que no depende de este problema. La duración total de un log tampoco
es la duración del experimento. Cada corrida contiene tres cosas distintas: la
caminata aleatoria, un tiempo muerto mientras yo envío el comando de congregación
a mano, y la congregación propiamente dicha. El tiempo muerto va de 0 a 46
segundos según la corrida, con una mediana de 20, y no es parte del experimento.
Ya separé las tres fases en el análisis, deduciendo el arranque de la
congregación a partir del momento en que los robots empiezan a pedir su posición,
que es algo que solo hacen en esa fase. Con eso, la duración de la congregación
queda entre 30 y 200 segundos según el escenario, y esos son los números
comparables entre topologías. Para las próximas sesiones voy a hacer que la base
registre sus propios comandos, así las fases quedan medidas y no deducidas.

Quedo atento a cualquier duda.
