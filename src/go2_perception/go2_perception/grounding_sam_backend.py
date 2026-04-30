from typing import Dict, List

import numpy as np


class GroundingSamBackend:
    """
    GroundingDINO + SAM2 fallback backend.
    This MVP keeps the interface stable and returns empty results if models are unavailable.
    """

    def __init__(self) -> None:
        self._available = False
        self._model = None
        try:
            # Placeholder for future integration.
            # Real setup can wire GroundingDINO and SAM2 pipelines here.
            self._available = False
        except Exception:
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    def infer(self, image_bgr: np.ndarray, text_prompt: str) -> List[Dict]:
        del image_bgr
        del text_prompt
        return []
