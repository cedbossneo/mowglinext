# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

import pytest

from backend_topics_test_common import BackendTopicsTestCase, generate_backend_test_description


@pytest.mark.launch_test
def generate_test_description():
    return generate_backend_test_description("mavros")


class TestMavrosBackendTopics(BackendTopicsTestCase):
    pass
