## Base Directory

Python code for computer running as swarm base.

System requierements:
- 1080p webcam placed on ceiling
- Python version ...

## Qué hay acá

En la raíz vive lo que corre durante un experimento, y nada más. Todo lo demás
está agrupado por para qué sirve.

| | |
|---|---|
| `AttaBot_Base.py` | el programa: cámara, ArUco, protocolo UDP y logging |
| `AttaBot_GUI.py` | su interfaz |
| `configSystem.json` | configuración del sistema de visión y de la red |
| `cameraMatrix.txt`, `dist.txt`, `mask.png` | calibración de la cámara |

`configSystem.json` nombra `cameraMatrix.txt` y `dist.txt` con rutas **relativas
al directorio de trabajo**, y la Base corre desde acá. Por eso esos tres archivos
se quedan en la raíz y no en una subcarpeta: moverlos rompe el arranque en el lab
salvo que se editen a la vez el JSON y `herramientas/Calibrar.py`, que es quien
los escribe.

### `analisis/` — el análisis del paper

| | |
|---|---|
| `analyze_logs.py` | métricas por corrida y por campaña; el motor de todo lo demás |
| `figuras_paper.py` | figuras del artículo y de revisión |
| `figura_escenarios.py` | planta de los escenarios, laboratorio y simulación |
| `extraer_geometria.py` | mide la geometría real desde los videos cenitales |
| `scan_logs.py` | escáner de anomalías sobre todos los logs |
| `pagina_revision.py` | página HTML para revisar una sesión |
| `campana_lab_30-07.csv` | manifiesto de la campaña del laboratorio |

Se corren desde `Base/`, no desde adentro de `analisis/`:

```sh
python analisis/analyze_logs.py --campaign analisis/campana_lab_30-07.csv
python analisis/figuras_paper.py paper
```

### `herramientas/` — banco, calibración y diagnóstico

| | |
|---|---|
| `check_system.py` | verifica cámara, red y dependencias |
| `ir_check.py` | infrarrojos: monitor, potenciómetro, umbral, motores |
| `turn_check.py` | mide el error de giro contra los logs de posición |
| `aruco_test.py` | detección de markers en vivo |
| `Calibrar.py` | calibración de cámara; escribe `cameraMatrix.txt` |
| `calibrar_colores.py` | umbrales de color |
| `marker_acrilico.py` | genera los SVG de corte de los markers |
| `test_network_config.py` | prueba de la configuración de red |
| `ValidacionRandomWalk.py` | validación del random walk |

### Directorios de datos

`PositionLogs/` y `ConsoleLogs/` guardan los logs de cada corrida. `Videos/` y
`Logs/` no se versionan. `analisis_30-07/` y `simulacion_31-07/` son los paquetes
de datos que se le pasaron a Juan Carlos, cada uno con su `LEEME.md`.
