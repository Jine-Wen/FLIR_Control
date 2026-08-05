"""Web console backend: HTTP + SSE transport layer (spec ARCHITECTURE.md
sec. 6, API.md).

Only ``server.py`` and ``sse.py`` live here, and neither imports ``rclpy``
or ``httpx`` -- the ROS glue is ``nodes/web_node.py``, which implements
:class:`~flir_ptz.webui.server.WebAdapter` (the duck-typed contract this
module calls into) and owns an :class:`~flir_ptz.webui.sse.SSEHub` that it
pushes into on every relevant ROS subscription callback.
"""

from flir_ptz.webui.server import ServerConfig, WebAdapter, make_server
from flir_ptz.webui.sse import SSEHub

__all__ = ["SSEHub", "ServerConfig", "WebAdapter", "make_server"]
