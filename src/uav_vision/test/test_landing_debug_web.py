#!/usr/bin/env python3
import pathlib
import sys
import unittest
import xml.etree.ElementTree as ET


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import landing_debug_web


class LatestJpegFrameTest(unittest.TestCase):
    def test_stores_latest_frame(self):
        frame = landing_debug_web.LatestJpegFrame()
        self.assertIsNone(frame.get())

        frame.update(b"first")
        frame.update(b"latest")

        self.assertEqual(frame.get(), b"latest")


class IndexHtmlTest(unittest.TestCase):
    def test_builds_page_with_custom_title(self):
        page = landing_debug_web.build_index_html(
            "UAV Target Classification Debug"
        ).decode("utf-8")

        self.assertIn(
            "<title>UAV Target Classification Debug</title>", page
        )
        self.assertIn(
            "<h1>UAV Target Classification Debug</h1>", page
        )

    def test_escapes_page_title(self):
        page = landing_debug_web.build_index_html(
            "<target>&debug"
        ).decode("utf-8")

        self.assertIn("&lt;target&gt;&amp;debug", page)
        self.assertNotIn("<h1><target>", page)


class TargetDebugWebLaunchTest(unittest.TestCase):
    def test_target_launch_uses_independent_topic_and_port(self):
        launch_path = (
            pathlib.Path(__file__).resolve().parents[1]
            / "launch"
            / "target_debug_web.launch"
        )
        root = ET.parse(str(launch_path)).getroot()
        args = {
            item.attrib["name"]: item.attrib["default"]
            for item in root.findall("arg")
        }
        node = root.find("node")

        self.assertEqual(
            args["image_topic"], "/vision/target/debug_image"
        )
        self.assertEqual(args["port"], "8081")
        self.assertEqual(
            args["page_title"], "UAV Target Classification Debug"
        )
        self.assertEqual(node.attrib["name"], "target_debug_web")
        self.assertEqual(node.attrib["type"], "landing_debug_web.py")


if __name__ == "__main__":
    unittest.main()
