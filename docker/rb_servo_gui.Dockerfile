FROM python:3.13-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    RB_GUI_HOST=0.0.0.0 \
    RB_GUI_PORT=8080 \
    RB_GUI_STATE_BIND=0.0.0.0 \
    RB_GUI_STATE_PORT=50110 \
    RB_GUI_COMMAND_HOST=rb_servo_server \
    RB_GUI_COMMAND_PORT=50010

WORKDIR /app
COPY gui/rb_servo_gui/pyproject.toml /app/pyproject.toml
COPY gui/rb_servo_gui/rb_servo_gui /app/rb_servo_gui
RUN pip install --no-cache-dir .
EXPOSE 8080/tcp 50110/udp
CMD ["python", "-m", "rb_servo_gui.app"]
