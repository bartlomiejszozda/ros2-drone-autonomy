"""
Send a chat to OpenRouter and return the model's answer.

OpenRouter offers many models behind one OpenAI-compatible endpoint, so changing the
model is only a different id string. The API key comes from the environment
(OPENROUTER_API_KEY), never from a parameter file that could end up in git.
"""

import requests

URL = 'https://openrouter.ai/api/v1/chat/completions'


class AssistantError(Exception):
    """The question could not be answered; the message says why."""


class OpenRouterClient:
    def __init__(self, api_key: str, timeout_sec: float, post=requests.post, url: str = URL):
        self._api_key = api_key
        self._timeout = timeout_sec
        self._post = post  # replaced by a fake in tests
        self._url = url

    def chat(self, model: str, messages: list[dict]) -> str:
        """Blocking: returns the answer text or raises AssistantError."""
        if not self._api_key:
            raise AssistantError('OPENROUTER_API_KEY is not set in the environment '
                                 'of the assistant node')
        try:
            response = self._post(
                self._url, timeout=self._timeout,
                headers={'Authorization': f'Bearer {self._api_key}',
                         'X-Title': 'Collider'},
                json={'model': model, 'messages': messages})
        except requests.Timeout:
            raise AssistantError(f'no answer within {self._timeout:g} s') from None
        except requests.RequestException as error:
            raise AssistantError(f'request failed: {error}') from None
        try:
            body = response.json()
        except ValueError:
            raise AssistantError(
                f'HTTP {response.status_code}: not JSON: {response.text[:200]}') from None
        # Errors come as {"error": {"message": ...}}, sometimes even with HTTP 200.
        if response.status_code != 200 or 'error' in body:
            error = body.get('error') or {}
            raise AssistantError(
                f'HTTP {response.status_code}: {error.get("message", response.text[:200])}')
        try:
            answer = body['choices'][0]['message']['content']
        except (KeyError, IndexError, TypeError):
            raise AssistantError(f'unexpected response: {str(body)[:200]}') from None
        if not answer:
            raise AssistantError('the model returned an empty answer')
        return answer
