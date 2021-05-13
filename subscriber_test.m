rosinit

arPoseSub = rossubscriber("/tf", @callback, "BufferSize", 15)

global msg


function callback(~,message)
msg = message;
msg.Transforms.ChildFrameId
end