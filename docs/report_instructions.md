# Final report: how to finish it

Due **December 11, 11:59 pm** on Canvas, one submission per team.
Status on 25 Nov 2026: every section is drafted from the repository's evidence.
Open items are marked *To fill* or *To confirm* in the draft.

## Where the report lives

| Copy | Location | Use it for |
|---|---|---|
| Live draft (editable, comments) | Claude Doc: https://claude.ai/artifact/5uYqA9Ci2y5inmMC4Udrmp | Editing and commenting. Private to Mateo until shared: use the doc's Share menu to add Zihao and Devin. |
| Snapshot | `docs/report_draft.md` | A copy in the repo, readable on GitHub. Refresh it from the live draft before submission. |
| Architecture figure | `docs/report_figures/architecture.png` | Figure for Section 3 |

## What the assignment requires (Canvas)

- A **6 to 10 page PDF**, single-spaced, **12-point font, 1-inch margins**; references and appendices do not count toward the limit.
- Content: problem, implemented algorithms, experimental setup, results, discussion, limitations, and **each member's contributions**.
- From the project slides: equations, experiment design, results, the extension, and contributions; related work with **at least three archival papers**.
- Alongside the PDF: the repository link (public), supporting materials (logs, plots, reproduction files), and the two-minute demonstration video.

## Steps to finish, in order

1. **Share the live draft** with Zihao and Devin (doc Share menu) so all three can edit and comment.
2. **Contributions (Section 10).** Each member fills their own row: code modules, experiments and analysis, presentation and video, report sections. Be specific, for example "`localization.py`: motion and measurement model, kidnap recovery". Each of you may be asked about any module.
3. **Lessons learned (end of Section 9).** Each member writes 2 to 4 sentences. Candidate points are already listed there.
4. **Related work and references (Section 2).**
   - Each member skims the papers you keep. Remove any you cannot stand behind.
   - Make sure at least three are archival papers outside the class readings. *Probabilistic Robotics* [5] may count as a class reading.
   - Check each reference's details (authors, venue, pages) against the published paper; the list was written from memory and needs that check.
5. **Figures.** Add 3 or 4 figures, each referenced in the text with a numbered caption:
   - Section 3: `docs/report_figures/architecture.png`
   - Section 8: `results/webots/figures/success_by_scenario.png`, plus `rmse_by_scenario.png` if space allows
   - Section 8 (extension): `results/webots/figures/extension_degradation.png`
   - Section 9: `results/webots/figures/kidnap_recovery.png`
6. **Check the numbers.** Every number in Section 8 should match `results/webots/summary.md`. If anyone reruns experiments, regenerate it (`python tools/summarize_webots.py`) and update the tables.
7. **Export and format.**
   - In the live draft, use the Export menu and choose **Word (.docx)**.
   - In Word: Times New Roman or similar at 12 pt, single spacing, 1-inch margins on all sides. Put the title block at the top of page 1 (no separate cover page is required).
   - Insert the figures from step 5 if the export did not carry them.
   - Equations export as LaTeX text. Retype them in Word's equation editor (Insert, Equation) so they render.
8. **Page count.** The body (Abstract through Section 10) must be 6 to 10 pages; References and Appendices A and B come after and do not count. If the body runs over 10 pages, trim in this order: Section 2, the extension table (keep the severe rows), then the per-paragraph detail in Section 4.
9. **Proofread** as a team: names, the repository link, consistent method names (`baseline`, `team`, `team_robust`), and no leftover *To fill* or *To confirm* markers.
10. **Save as PDF** from Word and submit on Canvas with:
    - the repository link: https://github.com/Norsedude/Autonomous-Indoor-Delivery
    - the two-minute demo video (cut from the clips in `demos/`; see `docs/demo_video.md`)
    - a note that logs, figures, seeds and analysis scripts are in the repository (`results/webots/`, `config/experiments.json`, `tools/`)
11. **Refresh the repo copy.** Export the final draft as Markdown to `docs/report_draft.md`, commit the final PDF as `docs/CMPE185_Group5_Final_Report.pdf`, and push.

## Also check

- **Peer evaluation:** the project assignment mentions one, but no form or location was given. Check Canvas or ask Dr. Wu.
- **Repository runs clean:** `python tools/validate_project.py` and `python -m unittest discover -s tests` should both pass on the submitted version.
