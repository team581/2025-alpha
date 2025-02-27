// pathflip if it sucked in every way but worked with trailblazer
// run with bun or run npm install and run with node

import { parseArgs } from "node:util";
import assert from "node:assert/strict";
import fs from "node:fs/promises";
import { multiReplace } from "@jonahsnider/util";

const args = parseArgs({
  options: {
    input: { type: "string", short: "i" },
  },
});

const { input: inputPath } = args.values;

assert(inputPath, "input is required");

const inputContents = await fs.readFile(inputPath, "utf8");

const FIELD_LENGTH = 17.55;
const POSE_2D_REGEXP = /new\s+Pose2d\((\d*\.\d+),\s*(\d*\.\d+)/g;
const ROTATION_2D_REGEXP = /Rotation2d\.fromDegrees\((\d+)(\.\d+)?\)/g;

function stupidRound(value, precision) {
  return Number(value.toFixed(precision));
}

function colorFlip(x, y) {
  const yAxis = FIELD_LENGTH / 2;
  return {
    x: yAxis - (x - yAxis),
    y,
  };
}

const TWO_PI = 2 * Math.PI;

/** Normalizes an angle to be within (-pi, pi]. */
function angleModulusRadians(angle) {
  const result = angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);

  if (result === -Math.PI) {
    return Math.PI;
  }

  return result;
}

function angleModulusDegrees(angle) {
  return angleModulusRadians((angle / 180) * Math.PI) * (180 / Math.PI);
}

function transformRotation(rotationDeg) {
  return angleModulusDegrees(-rotationDeg + 180);
}

function transformColors(value) {
  return multiReplace(value, {
    red: "blue",
    Red: "Blue",
    RED: "BLUE",
    blue: "red",
    Blue: "Red",
    BLUE: "RED",
  });
}

/** @param {string} text */
function transform(text) {
  const firstPass = text
    .replaceAll(POSE_2D_REGEXP, (_, xString, yString) => {
      const { x, y } = colorFlip(Number(xString), Number(yString));
      return `new Pose2d(${stupidRound(x, 3)}, ${stupidRound(y, 3)}`;
    })
    .replaceAll(ROTATION_2D_REGEXP, (_, degreesString) => {
      return `Rotation2d.fromDegrees(${stupidRound(
        transformRotation(Number(degreesString)),
        3
      )})`;
    });

  return transformColors(firstPass);
}

await fs.writeFile(transformColors(inputPath), transform(inputContents));
