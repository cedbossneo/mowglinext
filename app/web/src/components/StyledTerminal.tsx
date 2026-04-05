import styled from "styled-components";

export const StyledTerminal = styled.div`
  background: #141414;
  border-radius: 12px;
  overflow: hidden;

  div.react-terminal-wrapper {
    padding-top: 10px;
    height: 100%;
    background: #141414 !important;
  }

  div.react-terminal-wrapper > div.react-terminal-window-buttons {
    display: none;
  }

  div.react-terminal {
    font-size: 12px;
    line-height: 1.5;
  }
`;
