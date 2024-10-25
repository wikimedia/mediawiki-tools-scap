const start_backport_script = [
    {
        "type": "line",
        "line": "\u001b[32m15:17:05 Checking whether requested changes are in a branch deployed to production and their dependencies valid...\u001b[0m\n",
        "gap": 0.7514786720275879
      },
      {
        "type": "line",
        "line": "\u001b[32m15:17:05 Change '7' validated for backport\u001b[0m\n",
        "gap": 0.2587759494781494
      },
      {
        "type": "line",
        "line": "The following changes are scheduled for backport:\n┌───┬─────────────────────────────┬───────────┬─────────────────────────┐\n│ # │           Project           │   Branch  │ Subject                 │\n├───┼─────────────────────────────┼───────────┼─────────────────────────┤\n│ 7 │ operations/mediawiki-config │ train-dev │ group1 to 1.43.0-wmf.20 │\n└───┴─────────────────────────────┴───────────┴─────────────────────────┘\nBackport the changes? [y/N]: ",
        "gap": 0.000751495361328125
      },
      {
        "type": "interaction",
        "subtype": "choices",
        "prompt": "The following changes are scheduled for backport:\n┌───┬─────────────────────────────┬───────────┬─────────────────────────┐\n│ # │           Project           │   Branch  │ Subject                 │\n├───┼─────────────────────────────┼───────────┼─────────────────────────┤\n│ 7 │ operations/mediawiki-config │ train-dev │ group1 to 1.43.0-wmf.20 │\n└───┴─────────────────────────────┴───────────┴─────────────────────────┘\nBackport the changes?",
        "choices": {
          "Yes": "y",
          "No": "n"
        },
        "default": "n",
        "gap": 1.9073486328125e-06
      },    
]

export default start_backport_script