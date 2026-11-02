# APM-Lua

Conjunto de scripts Lua para ArduPilot / ArduCopter que amplían el comportamiento de vuelo durante la ejecución del firmware.

> ## ⚠️ Aviso de aplicabilidad
> **Todos los scripts Lua se han validado en aeronave real únicamente con la versión de firmware correspondiente; con cualquier otra versión de firmware pueden no surtir efecto.**

## Propósito del repositorio

Este repositorio aloja scripts Lua escritos para el firmware APM (ArduPilot / ArduCopter). Los scripts se ejecutan en el controlador de vuelo mediante el motor de scripts Lua integrado en ArduPilot y amplían o restringen el comportamiento de vuelo sin modificar el código fuente del firmware; por ejemplo, limitan dinámicamente la velocidad de descenso manual en función de la altitud de vuelo.

El repositorio contiene únicamente scripts que pueden copiarse directamente en el directorio `APM/scripts/` de la tarjeta SD del controlador de vuelo. No incluye código fuente del firmware, configuración de la estación de tierra ni dependencias de terceros.

## Versiones de firmware compatibles

- Firmware: ArduCopter 4.6.3 y 4.7.0 (multirrotor).
- Adaptación de parámetros: la versión 4.6.x utiliza `PILOT_SPEED_DN` / `PILOT_SPEED_UP` (unidad: cm/s); la versión 4.7.x utiliza `PILOT_SPD_DN` / `PILOT_SPD_UP` (unidad: m/s). Un adaptador integrado identifica el par automáticamente y exige que exista exactamente uno de los dos pares; de lo contrario, el script solo supervisa y nunca escribe.
- Otras versiones: no se implementa ninguna gestión de compatibilidad. Cualquier cambio en los nombres o en la semántica de los parámetros puede dejar los scripts sin efecto o provocar un comportamiento inesperado.

## Estructura de directorios

```text
.
├── LICENSE
├── README_CN.md / README_EN.md / README_FR.md / README_RU.md / README_ES.md / README_AR.md
└── scripts
    └── 近地面减速_near_ground_slowdown
        ├── near_ground_slowdown.lua
        ├── near_ground_slowdown_stage2_0p2.lua
        ├── near_ground_slowdown_smooth_10m_stage2_0p2.lua
        ├── near_ground_slowdown_smooth_10m_stage2_0p2.md
        └── README.md
```

## Inventario de scripts

| Script | Firmware | Resumen |
| --- | --- | --- |
| `near_ground_slowdown.lua` | ArduCopter 4.6.3 / 4.7.0 | En modos AltHold / Loiter limita la velocidad de descenso manual según la altura sobre el punto home: 0,50 m/s entre 2 m y 5 m, 0,10 m/s a 2 m o menos |
| `near_ground_slowdown_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | Igual que el anterior, pero el escalón ≤ 2 m se amplía a 0,20 m/s |
| `near_ground_slowdown_smooth_10m_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | Igual que el anterior, con un margen progresivo de 10 a 5 m (curva desde el valor original hasta 0,50 m/s) y cuantización en cm/s enteros para 4.6.x |

La definición de parámetros, los códigos de estado y los campos de registro de cada script se documentan en el `README.md` del directorio funcional correspondiente.

## Notas de uso

- Copie el script en el directorio `APM/scripts/` de la tarjeta SD del controlador de vuelo y compruebe que `SCR_ENABLE = 1`.
- Cargue un solo script por función; las tres variantes de `near_ground_slowdown` comparten la tabla de parámetros `NGC_` y no deben cargarse simultáneamente.
- Los scripts escriben únicamente parámetros de velocidad de descenso y no envían comandos de armado / desarmado / motor / modo; el aterrizaje se apoya en el desarmado automático nativo `DISARM_DELAY`.
- La altura procede de la posición EKF relativa al punto home, no de la altura real sobre el terreno; el relieve y la deriva barométrica afectan a la decisión.
- En el primer uso, retire las hélices y verifique en tierra el registro y la restauración de parámetros antes de volar.

## Licencia

Este proyecto se distribuye bajo la GNU General Public License v3.0. Consulte `LICENSE`.

## Idiomas

- 中文：[README_CN.md](README_CN.md)
- English：[README_EN.md](README_EN.md)
- Français：[README_FR.md](README_FR.md)
- Русский：[README_RU.md](README_RU.md)
- Español：[README_ES.md](README_ES.md)
- العربية：[README_AR.md](README_AR.md)
