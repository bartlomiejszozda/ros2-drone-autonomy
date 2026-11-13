"""Conversation memory sent with every question, and the history file on disk."""

import json
import os
from datetime import datetime


class Conversation:
    """
    Earlier questions and answers, as chat messages for the model.

    Kept in memory only, so a restarted node starts a new conversation. Only the text
    is remembered: an old image would be sent again with every later question and
    multiply the cost.
    """

    def __init__(self, max_turns: int, system_prompt: str = ''):
        self._max_turns = max_turns
        self._system_prompt = system_prompt
        self._turns: list[tuple[str, str]] = []

    @property
    def turns(self) -> int:
        return len(self._turns)

    def clear(self):
        self._turns.clear()

    def messages(self, question: str, image_url: str | None = None) -> list[dict]:
        """Everything to send: system prompt, earlier turns, then the new question."""
        messages = []
        if self._system_prompt:
            messages.append({'role': 'system', 'content': self._system_prompt})
        for asked, answered in self._turns:
            messages.append({'role': 'user', 'content': asked})
            messages.append({'role': 'assistant', 'content': answered})
        content = [{'type': 'text', 'text': question}]
        if image_url is not None:
            content.append({'type': 'image_url', 'image_url': {'url': image_url}})
        messages.append({'role': 'user', 'content': content})
        return messages

    def add(self, question: str, answer: str):
        self._turns.append((question, answer))
        # Drop the oldest turns; the model sees at most max_turns of them.
        del self._turns[:-self._max_turns]


class HistoryLog:
    """
    Append one JSON line per question to a file; an empty path turns it off.

    The file is only written, never read back, so it does not survive into the next
    conversation. Read it with `tail` or `jq`.
    """

    def __init__(self, path: str):
        self.path = os.path.expanduser(path) if path else ''
        if self.path:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)

    def append(self, **entry):
        if not self.path:
            return
        # Wall-clock time on purpose: a log entry should say when it really happened.
        record = {'time': datetime.now().isoformat(timespec='seconds'), **entry}
        with open(self.path, 'a', encoding='utf-8') as file:
            file.write(json.dumps(record, ensure_ascii=False) + '\n')
