# Assemble and mount

Check that each mechanism moves through its full range and remains seated during manipulation.

## Before assembly

Inspect all printed parts for:

- warped mounting faces;
- incomplete threads;
- support material inside holes or tracks;
- cracks around thin mechanisms; and
- loose press-fit interfaces.

Replace damaged functional parts rather than forcing them into place.

## Assemble the base

1. Place the honeycomb base on a flat surface.
2. Check that all seven cells are clear and dimensionally consistent.
3. Insert one attachment base and press evenly until it is fully seated.
4. Remove and reinsert the test attachment to confirm repeatable fit.
5. Repeat the seating check across the cells you intend to use.

## Assemble the mechanisms

The repository groups all parts required by each attachment in a dedicated STL folder. Keep the printed components from each folder together during assembly.

### Threaded parts

Engage the part by hand without axial forcing. It should advance smoothly while remaining constrained by the thread. If it binds, remove debris and test the matching calibration piece before sanding the final component.

### Ball valve and friction rings

Assemble the ball valve without a ring first. The ring configuration is evaluated as a separate attachment and should only be fitted for the corresponding trial block.

### Covered button

Confirm that the cover opens freely and that the button returns after it is pressed. Both stages must remain visually distinguishable.

### Drawer

Check the full pull-and-push travel. The drawer must not disengage unintentionally from its base during normal use.

### Lock and key

Check insertion and rotation manually. The initial key position must be repeatable across trials.

### Shock absorber

Confirm that the pin can be grasped, aligned, and fully inserted. This attachment occupies two adjacent cells; reserve both before arranging the board.

## Mount the board

Fix the base to a rigid surface within the platform's working volume. Horizontal tabletop and vertical fixture mounting are both allowed, but the orientation must be reported because it changes the approach direction and the likelihood of an attachment releasing from its cell.

Before recorded trials:

1. fully seat every attachment;
2. confirm that all modules are reachable without moving the base during a trial;
3. check for collision risks around the fixture;
4. photograph the setup as `setup.jpg`; and
5. describe the mounting orientation in `platform.md`.

::: warning Dynamic loading
High-gain or jittering commands can damage printed mechanisms or loosen press-fit attachments. Inspect the board before every session and after any contact that visibly displaces a module.
:::

Continue with the [evaluation protocol](/benchmark/protocol).
