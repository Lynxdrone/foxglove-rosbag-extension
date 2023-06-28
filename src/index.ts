import { ExtensionContext } from "@foxglove/studio";
import { initRosbagPanel } from "./RosbagPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Rosbag control", initPanel: initRosbagPanel });
}
