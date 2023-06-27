from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class ServoPosition(BaseModel):
    position: int


servo_position = 90  # Default servo position (0-180)


@app.get("/get_position")
def get_position():
    return {"position": servo_position}


@app.post("/set_position")
def set_position(position: ServoPosition):
    global servo_position
    servo_position = position.position
    return {"message": "Servo position updated", "position": servo_position}
