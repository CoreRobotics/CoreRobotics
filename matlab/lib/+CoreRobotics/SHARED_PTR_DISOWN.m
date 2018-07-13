function v = SHARED_PTR_DISOWN()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 0);
  end
  v = vInitialized;
end
