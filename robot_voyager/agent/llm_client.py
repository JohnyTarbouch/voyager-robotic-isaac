import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

import httpx

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LLMConfig:
    base_url: str
    api_key: str 
    model: str


class LLMClient:
    def __init__(self, cfg: LLMConfig, timeout: float = 120.0):
        """
        Initialize the LLM client.
        
        Args:
            cfg: LLM configuration
            timeout: Request timeout
        """
        base = (cfg.base_url or "").rstrip("/")
        if not base:
            raise RuntimeError("No base_url. Set LLM_BASE_URL")
        
        self.cfg = cfg
        self.url = f"{base}/chat/completions"
        
        # Set up headers
        self.headers = {"Content-Type": "application/json"}
        if cfg.api_key and cfg.api_key.lower() not in ("none", ""):
            self.headers["Authorization"] = f"Bearer {cfg.api_key}"
        
        self._client = httpx.Client(timeout=httpx.Timeout(timeout))
        logger.info(f"LLM client initialized: {base} (model: {cfg.model})")
        
        # Logs callback
        self._metrics_callback: Optional[Callable] = None
    
    def set_metrics_callback(self, callback: Callable):
        self._metrics_callback = callback

    def chat(
        self, 
        messages: List[Dict[str, Any]], 
        *, 
        temperature: float = 0.2, 
        max_tokens: int = 1024,
        stop: Optional[List[str]] = None,
        call_type: str = "unknown",
    ) -> str:
        """
        Send a chat completion request.
        
        Args:
            messages: List of message dicts
            temperature: Sampling temperature
            max_tokens: Maximum tokens in resp
            stop: Optional stop sequences
            call_type: Type of call for metrics logging
        """
        payload = {
            "model": self.cfg.model,
            "messages": messages,
            "temperature": float(temperature),
            "max_tokens": int(max_tokens),
        }
        if stop:
            payload["stop"] = stop
        
        # Extract prompts for logging
        system_prompt = ""
        user_prompt = ""
        for msg in messages:
            if msg["role"] == "system":
                system_prompt = msg["content"]
            elif msg["role"] == "user":
                user_prompt = msg["content"]
        
        start_time = time.time()
        
        try:
            response = self._client.post(self.url, headers=self.headers, json=payload)
            response.raise_for_status()
            data = response.json()
            content = data["choices"][0]["message"].get("content") or ""
            content = content.strip()
            
            latency_ms = (time.time() - start_time) * 1000
            
            # Extract token counts if available
            usage = data.get("usage", {})
            prompt_tokens = usage.get("prompt_tokens")
            completion_tokens = usage.get("completion_tokens")
            
            # Log to metrics if callback set
            if self._metrics_callback:
                self._metrics_callback(
                    call_type=call_type,
                    system_prompt=system_prompt,
                    user_prompt=user_prompt,
                    response_text=content,
                    latency_ms=latency_ms,
                    temperature=temperature,
                    max_tokens=max_tokens,
                    prompt_tokens=prompt_tokens,
                    completion_tokens=completion_tokens,
                )
            
            return content
            
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error {e.response.status_code}: {e.response.text}")
            raise
        except Exception as e:
            logger.error(f"LLM request failed: {e}")
            raise

    def chat_json(
        self, 
        messages: List[Dict[str, Any]], 
        **kwargs
    ) -> Dict[str, Any]:
        """
        Request JSON-formatted response.
        """
        if messages and messages[0]["role"] == "system":
            messages = messages.copy()
            messages[0] = {
                "role": "system",
                "content": messages[0]["content"] + "\n\nRespond with valid JSON only."
            }
        
        response = self.chat(messages, **kwargs)
        
        # Parse JSON from response
        import json
        
        if "```json" in response:
            response = response.split("```json")[1].split("```")[0]
        elif "```" in response:
            response = response.split("```")[1].split("```")[0]
        
        return json.loads(response.strip())

    def close(self) -> None:
        self._client.close()
