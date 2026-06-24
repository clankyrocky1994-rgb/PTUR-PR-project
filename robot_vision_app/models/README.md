# Models

Сюда складываются веса нейросетей. Оба файла уже включены в проект; если их
нет — они скачиваются автоматически при первом запуске.

| Файл | Назначение | Размер | Источник |
|------|-----------|--------|----------|
| `yolo11n-pose.pt` | Поза тела (плечо-локоть-запястье) | ~6 МБ | Ultralytics YOLO11 |
| `hand_landmarker.task` | 21 точка кисти | ~7.5 МБ | Google MediaPipe |

## Откуда берутся

**MediaPipe** — приложение скачивает модель автоматически в эту папку при
первом запуске (см. `MediaPipeHandDetector._download_model`). Прямая ссылка:

```
https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

**YOLO11** — Ultralytics скачивает веса по имени модели автоматически, если файл
не найден. Можно скачать заранее:

```python
from ultralytics import YOLO
YOLO("yolo11n-pose.pt")   # положит файл в текущую папку, перенесите в models/
```

## Лицензии

- YOLO11 — Ultralytics (AGPL-3.0 / коммерческая), https://ultralytics.com
- MediaPipe — Google (Apache-2.0), https://developers.google.com/mediapipe

## Замечание про размер репозитория

Веса — крупные бинарные файлы. В `.gitignore` есть закомментированные строки,
чтобы исключить их из git-истории, если вы захотите хранить код отдельно от
весов (например, через [Git LFS](https://git-lfs.com/)).
