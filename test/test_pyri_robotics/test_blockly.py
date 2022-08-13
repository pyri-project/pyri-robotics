from pyri.sandbox.util import run_blockly_compile_test

def test_blockly_compiler_robotics():
    robotics_blockly_json = \
"""
{
  "blocks": {
    "languageVersion": 0,
    "blocks": [
      {
        "type": "procedures_defnoreturn",
        "id": "aaaaa",
        "x": 20,
        "y": 20,
        "icons": {
          "comment": {
            "text": "Describe this function...",
            "pinned": false,
            "height": 80,
            "width": 160
          }
        },
        "fields": {
          "NAME": "my_procedure"
        },
        "inputs": {
          "STACK": {
            "block": {
              "type": "robot_movej",
              "id": "sa8g;sal?f%|pj-rY@^3",
              "fields": {
                "SPEED": 100,
                "WAIT": true
              },
              "inputs": {
                "JOINT_VECTOR": {
                  "block": {
                    "type": "variables_get",
                    "id": "+?Nc$G?(J8Qqx[A3~-Z=",
                    "fields": {
                      "VAR": {
                        "id": "D%*hl$cI_^kaTc,D)pCk"
                      }
                    }
                  }
                }
              },
              "next": {
                "block": {
                  "type": "robot_movel",
                  "id": "QaY0CQ`n2h_8hG_BI=?}",
                  "fields": {
                    "SPEED": 100,
                    "FRAME": "WORLD",
                    "WAIT": true
                  },
                  "inputs": {
                    "ROBOT_POSE": {
                      "block": {
                        "type": "variables_get",
                        "id": "y000Y2`5yld$|yAPGE3Q",
                        "fields": {
                          "VAR": {
                            "id": "D%*hl$cI_^kaTc,D)pCk"
                          }
                        }
                      }
                    }
                  },
                  "next": {
                    "block": {
                      "type": "variables_set",
                      "id": "uCX@./#F_Pn|hoQ[zpE!",
                      "fields": {
                        "VAR": {
                          "id": "D%*hl$cI_^kaTc,D)pCk"
                        }
                      },
                      "inputs": {
                        "VALUE": {
                          "block": {
                            "type": "robot_get_end_pose",
                            "id": "2:3[^NDF~LRYl}UvpQx:",
                            "fields": {
                              "FRAME": "WORLD"
                            }
                          }
                        }
                      },
                      "next": {
                        "block": {
                          "type": "variables_set",
                          "id": "#Eiz0RqCP@Q;O:XN-tK2",
                          "fields": {
                            "VAR": {
                              "id": "D%*hl$cI_^kaTc,D)pCk"
                            }
                          },
                          "inputs": {
                            "VALUE": {
                              "block": {
                                "type": "robot_get_joint_position",
                                "id": "=]{3B@Q4:kCHTote1;Bh"
                              }
                            }
                          },
                          "next": {
                            "block": {
                              "type": "robot_tool_gripper",
                              "id": "s3!*ylA-+1eDzyqfXLnD",
                              "fields": {
                                "GRIPPER_STATUS": "0"
                              },
                              "next": {
                                "block": {
                                  "type": "robot_planar_grab",
                                  "id": "s%$qS0B?yBynlzRxkG~h",
                                  "fields": {
                                    "REFERENCE_POSE": "reference_pose_global_aaa",
                                    "SPEED": 100,
                                    "WAIT": true
                                  },
                                  "inputs": {
                                    "Z_BEFORE": {
                                      "block": {
                                        "type": "variables_get",
                                        "id": "+xN]h5Mr*PH94ElT8:]V",
                                        "fields": {
                                          "VAR": {
                                            "id": "?Ze.h9idds(oyHu@Ksh%"
                                          }
                                        }
                                      }
                                    },
                                    "Z_GRAB": {
                                      "block": {
                                        "type": "variables_get",
                                        "id": "N$zWgh0u?M3-?uTxjML@",
                                        "fields": {
                                          "VAR": {
                                            "id": "3vH`_~O-qqQG$H):;+6F"
                                          }
                                        }
                                      }
                                    },
                                    "OBJECT_POSE": {
                                      "block": {
                                        "type": "variables_get",
                                        "id": "qTwhZv6ihmrZT.EAl(+F",
                                        "fields": {
                                          "VAR": {
                                            "id": "$nWQ|,pfhi9N:=YHY-LU"
                                          }
                                        }
                                      }
                                    }
                                  },
                                  "next": {
                                    "block": {
                                      "type": "robot_planar_place",
                                      "id": "{zbE~|@2wiB_u(YRBl-m",
                                      "fields": {
                                        "REFERENCE_POSE": "reference_pose_global_bbb",
                                        "SPEED": 100,
                                        "WAIT": true
                                      },
                                      "inputs": {
                                        "Z_BEFORE": {
                                          "block": {
                                            "type": "variables_get",
                                            "id": "zXa_S%Dm7q(+-Um{r6Mo",
                                            "fields": {
                                              "VAR": {
                                                "id": "?Ze.h9idds(oyHu@Ksh%"
                                              }
                                            }
                                          }
                                        },
                                        "Z_PLACE": {
                                          "block": {
                                            "type": "variables_get",
                                            "id": "3NAMfz-|ZqL{K,Z]1c`[",
                                            "fields": {
                                              "VAR": {
                                                "id": "3vH`_~O-qqQG$H):;+6F"
                                              }
                                            }
                                          }
                                        },
                                        "TARGET_POSE": {
                                          "block": {
                                            "type": "variables_get",
                                            "id": "pmzi=2,x+VTh{~1Ji}W^",
                                            "fields": {
                                              "VAR": {
                                                "id": "$nWQ|,pfhi9N:=YHY-LU"
                                              }
                                            }
                                          }
                                        }
                                      },
                                      "next": {
                                        "block": {
                                          "type": "robot_set_active_robot",
                                          "id": "1h[{I}=Emgi9_ipmowjF",
                                          "fields": {
                                            "ROBOT_NAME": "robot_ccc"
                                          },
                                          "next": {
                                            "block": {
                                              "type": "robot_set_origin_calibration",
                                              "id": "GnbO6fY%Tq81[G.k2T7M",
                                              "fields": {
                                                "ROBOT_CALIBRATION_NAME": "robot_origin_calibration_ddd"
                                              },
                                              "next": {
                                                "block": {
                                                  "type": "robot_set_active_tool",
                                                  "id": "suxY2OD,YMNXl!RwVopf",
                                                  "fields": {
                                                    "TOOL_NAME": "tool_eee"
                                                  }
                                                }
                                              }
                                            }
                                          }
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    ]
  },
  "variables": [
    {
      "name": "var1",
      "id": "D%*hl$cI_^kaTc,D)pCk"
    },
    {
      "name": "var2",
      "id": "?Ze.h9idds(oyHu@Ksh%"
    },
    {
      "name": "var3",
      "id": "3vH`_~O-qqQG$H):;+6F"
    },
    {
      "name": "var4",
      "id": "$nWQ|,pfhi9N:=YHY-LU"
    }
  ]
}
"""
    expected_pysrc = \
        "# Describe this function...\n" \
        "def my_procedure():\n" \
        "\n" \
        "  robot_movej(var1, float(100), True)\n" \
        "  robot_movel(var1, \"WORLD\", float(100), True)\n" \
        "  var1 = robot_get_end_pose(\"WORLD\")\n" \
        "  var1 = robot_get_joint_position()\n" \
        "  robot_tool_gripper(\"0\")\n" \
        "  robot_planar_grab(var4, \"reference_pose_global_aaa\", var2, var3, float(100), True)\n" \
        "  robot_planar_place(var4, \"reference_pose_global_bbb\", var2, var3, float(100), True)\n" \
        "  robot_set_active_robot(\"robot_ccc\")\n" \
        "  robot_set_origin_calibration(\"robot_origin_calibration_ddd\")\n" \
        "  robot_set_active_tool(\"tool_eee\")\n" \

    run_blockly_compile_test(robotics_blockly_json, expected_pysrc)