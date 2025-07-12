# YOLOv13 FastAPI REST API

**What is this?**  
A REST API server that detects objects in images using YOLOv13 AI models. Upload an image, get back detection results with bounding boxes and confidence scores.

**Key Benefits:**
- Real-time detection (~6.9 FPS with YOLOv13n)
- Multiple YOLO model support (YOLOv13, YOLOv8)
- Simple REST API interface
- Production-ready with error handling

## Quick Start
Before starting the server, make sure you have installed this extra requirement: 

```bash
pip install huggingface-hub
```

Then, start the server:

```bash
# Install dependencies
pip install -r requirements.txt

# Start the server
python yolov13_fastapi_api.py
```

Server runs at: http://localhost:8000  
API docs: http://localhost:8000/docs

## Usage

### Basic Detection

```bash
curl -X POST "http://localhost:8000/detect" \
     -F "image=@your_image.jpg" \
     -F "model=yolov13n"
```

### With Custom Settings

```bash
curl -X POST "http://localhost:8000/detect" \
     -F "image=@your_image.jpg" \
     -F "model=yolov13n" \
     -F "conf=0.25" \
     -F "iou=0.45"
```

### Get Available Models

```bash
curl http://localhost:8000/models
```

## Available Models

- **YOLOv13**: yolov13n, yolov13s, yolov13m, yolov13l, yolov13x
- **YOLOv8**: yolov8n, yolov8s, yolov8m, yolov8l, yolov8x

**Recommended for real-time**: yolov13n (fastest)

## Response Format

```json
{
  "success": true,
  "model_used": "yolov13n",
  "inference_time": 0.146,
  "detections": [
    {
      "bbox": [x1, y1, x2, y2],
      "confidence": 0.85,
      "class_id": 0,
      "class_name": "person"
    }
  ],
  "num_detections": 1,
  "image_info": {
    "width": 640,
    "height": 480,
    "channels": 3
  }
}
```

## Deployment

### Docker Deployment

```bash
# Build image
docker build -t yolov13-api .

# Run container
docker run -p 8000:8000 yolov13-api
```

### Docker Compose

```yaml
version: '3.8'
services:
  yolov13-api:
    build: .
    ports:
      - "8000:8000"
    volumes:
      - ./models:/app/models  # Optional: for custom models
```

### Production Deployment

```bash
# Install production server
pip install gunicorn

# Run with gunicorn
gunicorn -w 4 -k uvicorn.workers.UvicornWorker main:app --bind 0.0.0.0:8000
```

### Environment Variables

```bash
export MODEL_PATH=/path/to/custom/model.pt  # Optional
export API_HOST=0.0.0.0
export API_PORT=8000
```

## Performance

- **YOLOv13n**: ~0.146s inference (~6.9 FPS)
- **YOLOv8n**: ~0.169s inference (~5.9 FPS)

YOLOv13n is **13.5% faster** than YOLOv8n with identical accuracy. 
