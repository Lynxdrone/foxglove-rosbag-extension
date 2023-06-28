import { Immutable, Topic, MessageEvent, PanelExtensionContext, SettingsTreeAction, SettingsTreeNodes, SettingsTreeNode, SettingsTreeFields } from "@foxglove/studio";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import ReactDOM from "react-dom";
import { set } from 'lodash';
import filesize from 'filesize.js'
import humanizeDuration from 'humanize-duration'

// import { ros1 } from "@foxglove/rosmsg-msgs-common";

type UInt32Message = { data: number };
type ServiceResponse = { success: boolean, message: string };
type Duration = { sec: number, nsec: number };
type DurationMsg = { data: Duration };

// Panel config with all the inner parameters
type Config = {
  bagName: string;
  saveDir: string;
  istopicToRecord: boolean[]
  topicsToRecord: string[]
};

// Generate settings tree for settings API
function buildSettingsTree(config: Config, topics: readonly Topic[]): SettingsTreeNodes {
  const general: SettingsTreeNode = {
    label: "Bag settings",
    icon: "Settings",
    fields: {
      saveDir: {
        label: "Save directory",
        // (on machine running the data_recording node)
        input: "string",
        value: config.saveDir,
        help: "on machine running the data_recording node",
      },
      bagName: {
        label: "Bag name",
        input: "string",
        value: config.bagName,
        placeholder: "my_bag",
      }
    },
  };


  // Generate a boolean field for each topic
  const fields: SettingsTreeFields = {};
  for (let i = 0; i < topics.length; i++) {
    fields["topic" + i.toString()] = {
      label: topics.map((t) => t.name)[i] as string,
      input: "boolean",
      value: config.istopicToRecord[i]
    };
  }

  const topic_selection: SettingsTreeNode = {
    label: "Topic selection",
    icon: "Topic",
    fields,
  };

  return { general, topic_selection };
}

function RosbagPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const { saveState } = context;
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();

  const [duration, setduration] = useState("");
  const [bagSize, setBagSize] = useState<number | undefined>();
  const [isRecording, setisRecording] = useState(false);

  // Init config variable
  const [config, setConfig] = useState<Config>(() => {
    const partialConfig = context.initialState as Config;

    const {
      bagName = "",
      saveDir = "~/Documents/rosbag/test",
      istopicToRecord = [],
      topicsToRecord = [],
    } = partialConfig;

    return { bagName, saveDir, istopicToRecord, topicsToRecord, };
  });


  // Main Layout effect
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      setMessages(renderState.currentFrame);
      setTopics(renderState.topics);
    };

    context.watch("topics");
    context.watch("currentFrame");
    context.subscribe([{ topic: "/data_recording/bag_size" }, { topic: "/data_recording/duration" }]);

  }, [context]);


  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Settings functions
  // Decide what to do when a setting is updated
  const settingsActionHandler = useCallback((action: SettingsTreeAction) => {
    if (action.action !== "update") {
      return;
    }

    // Update config based on the previous state
    setConfig((previous) => {
      const newConfig = { ...previous };
      set(newConfig, action.payload.path.slice(1), action.payload.value);

      // Update istopicToRecord array and publish topics to record as a ROS parameter
      const topicsToRecord: string[] = [];
      for (let i = 0; i < (topics ?? []).length; i++) {
        if (action.payload.path[0] === "topic_selection" && action.payload.path[1] === ("topic" + i.toString())) {
          newConfig.istopicToRecord[i] = action.payload.value as boolean;
        }
        // Add topic name to the list if toggle is on
        if (newConfig.istopicToRecord[i]) {
          topicsToRecord.push((topics ?? []).map((t) => t.name)[i] as string)
        }
      }
      context.setParameter("/data_recording/topics", topicsToRecord);
      return newConfig;
    });
  }, [topics]);

  // Update settings panel to layout
  useEffect(() => {
    const tree = buildSettingsTree(config, topics ?? []);
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: tree,
    });
    saveState(config);
  }, [config, context, saveState, settingsActionHandler, topics]);

  // Watch for changes to the config.bagName variable and update the ROS parameter
  useEffect(() => {
    if (config.bagName) {
      // Don't change parameter if field is empty
      context.setParameter("/data_recording/bag_name", config.bagName);
    }
  }, [config.bagName]);

  // Watch for changes to the config.saveDir variable and update the ROS parameter
  useEffect(() => {
    if (config.saveDir) {
      // Don't change parameter if field is empty
      context.setParameter("/data_recording/output_directory", config.saveDir);
    }
  }, [config.saveDir]);

  // Watch for changes to the messages prop and get data from the wanted topics
  useEffect(() => {
    if (messages) {
      const bagSizeMsg = messages.find((msg) => msg.topic === "/data_recording/bag_size") as MessageEvent<UInt32Message>;
      if (bagSizeMsg) {
        setBagSize(bagSizeMsg.message.data as number);
      }

      const bagDurationMsg = messages.find((msg) => msg.topic === "/data_recording/duration") as MessageEvent<DurationMsg>;
      if (bagDurationMsg) {
        if (bagDurationMsg.message.data.sec > 0) {
          // Detect if bag was already recording before launching foxglove
          setisRecording(true);
        }

        const durationString = humanizeDuration(
          bagDurationMsg.message.data.sec * 1000 + bagDurationMsg.message.data.nsec / 1000000,
          { maxDecimalPoints: 2 });
        setduration(durationString)
      }
    }
  }, [messages]);


  // Handler function to start recording a new rosbag
  const handleStartRecording = useCallback(
    async () => {
      if (!context.callService) {
        return;
      }

      try {
        const response = await context.callService("/data_recording/start_recording", {}) as ServiceResponse;
        if (response.success) {
          // Change recording state only if service call is successful
          setisRecording(true);
          saveState(config);
        }
      } catch (error) {
        console.error(error);
      }
    },
    [context.callService, config, saveState],
  );

  // Handler function to stop the current rosbag recording
  const handleStopRecording = useCallback(
    async () => {
      if (!context.callService) {
        return;
      }

      try {
        const response = await context.callService("/data_recording/stop_recording", {}) as ServiceResponse;
        if (response.success) {
          // Change recording state only if service call is successful
          setisRecording(false);
          saveState(config);
        }
      } catch (error) {
        console.error(error);
      }
    },
    [context.callService, config, saveState],
  );


  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Render the panel UI
  return (
    <div
    style={{
      padding: "1rem",
      borderRadius: "0.5rem",
    }}
  >
    {context.callService == undefined && (
      <p style={{ color: "red", fontWeight: "bold", marginBottom: "1rem" }}>
        Calling services is not supported by this connection
      </p>
    )}
    <div
      style={{
        display: "grid",
        gridTemplateColumns: "repeat(2, 1fr)",
        gap: "0.5rem",
        marginBottom: "1rem",
        alignItems: "center",
      }}
    >
      <div
        style={{
          backgroundColor: "#3a3a3e",
          color: "#ffffff",
          padding: "1rem",
          borderRadius: "0.5rem",
          textAlign: "center",
        }}
      >
        <div
          style={{
            fontWeight: "bold",
            marginBottom: "0.5rem",
          }}
        >
          Bag Size
        </div>
        <div>{filesize(bagSize ?? 0)}</div>
      </div>
      <div
        style={{
          backgroundColor: "#3a3a3e",
          color: "#ffffff",
          padding: "1rem",
          borderRadius: "0.5rem",
          textAlign: "center",
        }}
      >
        <div
          style={{
            fontWeight: "bold",
            marginBottom: "0.5rem",
          }}
        >
          Duration
        </div>
        <div>{duration}</div>
      </div>
    </div>
    <button
      onClick={async () => {
        await isRecording ? handleStopRecording() : handleStartRecording();
      }}
      style={{
        padding: "0.5rem",
        borderRadius: "0.5rem",
        backgroundColor: isRecording ? "red" : "green",
        color: "white",
        fontWeight: "bold",
        border: "none",
        cursor: "pointer",
        width: "100%",
      }}
    >
      {isRecording ? "Stop Recording" : "Start Recording"}
    </button>
  </div>
  );
}

// Entry point for the panel extension
export function initRosbagPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<RosbagPanel context={context} />, context.panelElement);

  // Return a cleanup function to remove the component when the panel is closed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}