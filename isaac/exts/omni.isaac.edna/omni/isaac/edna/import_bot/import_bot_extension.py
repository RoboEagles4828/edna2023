# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import os
from omni.isaac.edna.base_sample import BaseSampleExtension
from omni.isaac.edna.import_bot.import_bot import ImportBot
import omni.ui as ui
from omni.isaac.ui.ui_utils import state_btn_builder


class ImportBotExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="Edna FRC",
            submenu_name="",
            name="Import URDF",
            title="Load the URDF for Edna FRC 2023 Robot",
            doc_link="",
            overview="This loads the Edna robot into Isaac Sim.",
            file_path=os.path.abspath(__file__),
            sample=ImportBot(),
            number_of_extra_frames=1
        )
        super()._on_setup_digital_twin()
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        return

    def _on_toggle_camera_button_event(self, val):
        return

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True

                dict = {
                    "label": "Toggle Cameras",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "Start the cameras",
                    "on_clicked_fn": self._on_toggle_camera_button_event,
                    
                }
                self.task_ui_elements["Toggle Camera"] = state_btn_builder(**dict)
                self.task_ui_elements["Toggle Camera"].enabled = False
