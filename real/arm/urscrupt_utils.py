    def _send_urscript(self, prog):
        """
        Method to send URScript program to the URScript ROS topic.

        Args:
            prog (str): URScript program which will be sent and run on
                the UR5e machine.

        """
        self._urscript_pub.publish(prog)

    def _output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant.

        Args:
            msg (str): message to display.

        """
        prog = 'textmsg(%s)' % msg
        self._send_urscript(prog)

    def _get_tip_transform(self):
        """
        Internal method to get the transform between the robot's
        wrist and the tip of the gripper.

        Returns:
            2-element tuple containing

            - list: Translation component of the gripper tip transform
              (shape :math:`[3,]`).
            - list: Euler angle orientation component of the gripper
              tip transform. (shape :math:`[3,]`).
        """
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        gripper_tip_id = self.arm_link_names.index(ee_frame)
        gripper_tip_link = self._urdf_chain.getSegment(gripper_tip_id)
        gripper_tip_tf = kdl_frame_to_numpy(gripper_tip_link.getFrameToTip())
        gripper_tip_pos = gripper_tip_tf[:3, 3].flatten()
        gripper_tip_rot_mat = gripper_tip_tf[:3, :3]
        gripper_tip_euler = arutil.rot2euler(gripper_tip_rot_mat)
        return list(gripper_tip_pos), list(gripper_tip_euler)

    def _set_tool_offset(self):
        """
        Internal method to send a URScript command to the robot so that
        it updates it tool center point variable to match the URDF.
        """
        tool_offset_prog = 'set_tcp(p[%f, %f, %f, %f, %f, %f])' % (
            self.gripper_tip_pos[0],
            self.gripper_tip_pos[1],
            self.gripper_tip_pos[2],
            self.gripper_tip_ori[0],
            self.gripper_tip_ori[1],
            self.gripper_tip_ori[2]
        )

        self._output_pendant_msg(tool_offset_prog)
        self._send_urscript(tool_offset_prog)
