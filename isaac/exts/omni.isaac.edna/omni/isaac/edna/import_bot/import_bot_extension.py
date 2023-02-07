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
        )
        return
